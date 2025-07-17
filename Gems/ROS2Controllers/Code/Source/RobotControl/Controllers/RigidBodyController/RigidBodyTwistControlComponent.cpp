/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "RigidBodyTwistControlComponent.h"
#include <AzCore/Component/TransformBus.h>
#include <AzCore/Math/MathUtils.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>
#include <AzFramework/Physics/RigidBodyBus.h>
#include <AzFramework/Physics/SimulatedBodies/RigidBody.h>

namespace ROS2Controllers
{
    void RigidBodyTwistControlComponent::Reflect(AZ::ReflectContext* context)
    {
        RigidBodyTwistControlComponentConfig::Reflect(context);

        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<RigidBodyTwistControlComponent, AZ::Component>()->Version(1)->Field(
                "Config", &RigidBodyTwistControlComponent::m_config);
            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<RigidBodyTwistControlComponent>("Rigid Body Twist Control", "Simple control through RigidBody")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->Attribute(AZ::Edit::Attributes::Icon, "Editor/Icons/Components/RigidBodyTwistControl.svg")
                    ->Attribute(AZ::Edit::Attributes::ViewportIcon, "Editor/Icons/Components/Viewport/RigidBodyTwistControl.svg")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &RigidBodyTwistControlComponent::m_config,
                        "Configuration",
                        "Configuration for the Rigid Body Twist Control Component")
                    ->Attribute(AZ::Edit::Attributes::Visibility, AZ::Edit::PropertyVisibility::ShowChildrenOnly);
            }
        }
    }

    void RigidBodyTwistControlComponent::Activate()
    {
        AZ::TickBus::Handler::BusConnect();
        TwistNotificationBus::Handler::BusConnect(GetEntityId());
        for (auto& controller : m_config.m_linerControllers)
        {
            controller.InitializePid();
        }
        for (auto& controller : m_config.m_angularControllers)
        {
            controller.InitializePid();
        }
    }

    void RigidBodyTwistControlComponent::Deactivate()
    {
        TwistNotificationBus::Handler::BusDisconnect();
        if (m_sceneFinishSimHandler.IsConnected())
        {
            m_sceneFinishSimHandler.Disconnect();
        }
        if (AZ::TickBus::Handler::BusIsConnected())
        {
            AZ::TickBus::Handler::BusDisconnect();
        }
    }

    void RigidBodyTwistControlComponent::OnTick([[maybe_unused]] float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
        AzPhysics::SceneInterface* sceneInterface = AZ::Interface<AzPhysics::SceneInterface>::Get();
        AZ_Assert(sceneInterface, "No scene interface");
        if (!sceneInterface)
        {
            return;
        }
        AzPhysics::SceneHandle defaultSceneHandle = sceneInterface->GetSceneHandle(AzPhysics::DefaultPhysicsSceneName);
        AZ_Assert(defaultSceneHandle != AzPhysics::InvalidSceneHandle, "Invalid default physics scene handle");
        AzPhysics::RigidBody* rigidBody = nullptr;
        Physics::RigidBodyRequestBus::EventResult(rigidBody, GetEntityId(), &Physics::RigidBodyRequests::GetRigidBody);
        AZ_Warning("RigidBodyTwistControlComponent", rigidBody, "No rigid body found for entity %s", GetEntity()->GetName().c_str());
        if (!rigidBody)
        {
            return;
        }
        m_bodyHandle = rigidBody->m_bodyHandle;
        m_sceneFinishSimHandler = AzPhysics::SceneEvents::OnSceneSimulationFinishHandler(
            [this]([[maybe_unused]] AzPhysics::SceneHandle sceneHandle, float fixedDeltaTime)
            {
                OnSceneSimulationFinish(sceneHandle, fixedDeltaTime);
            },
            aznumeric_cast<int32_t>(AzPhysics::SceneEvents::PhysicsStartFinishSimulationPriority::Components));
        sceneInterface->RegisterSceneSimulationFinishHandler(defaultSceneHandle, m_sceneFinishSimHandler);
        AZ::TickBus::Handler::BusDisconnect();
    }

    AZ::Vector3 ComputeImpulse(const AZ::Vector3& errorVec, AZStd::array<PidConfiguration, 3>& controllers, float fixedDeltaTime)
    {
        AZ::Vector3 impulse = AZ::Vector3::CreateZero();
        for (size_t i = 0; i < 3; ++i)
        {
            const auto error = errorVec.GetElement(i);
            impulse.SetElement(i, controllers[i].ComputeCommand(error, fixedDeltaTime * 1'000'000'000));
        }
        return impulse;
    }

    void RigidBodyTwistControlComponent::OnSceneSimulationFinish(AzPhysics::SceneHandle sceneHandle, float fixedDeltaTime)
    {
        AzPhysics::SceneInterface* sceneInterface = AZ::Interface<AzPhysics::SceneInterface>::Get();
        AZ_Assert(sceneInterface, "No scene interface");

        AzPhysics::SceneHandle defaultSceneHandle = sceneInterface->GetSceneHandle(AzPhysics::DefaultPhysicsSceneName);
        AZ_Assert(defaultSceneHandle != AzPhysics::InvalidSceneHandle, "Invalid default physics scene handle");

        if (m_bodyHandle == AzPhysics::InvalidSimulatedBodyHandle)
        {
            // This component is only interested in the default scene
            return;
        }
        auto* rigidBody = sceneInterface->GetSimulatedBodyFromHandle(sceneHandle, m_bodyHandle);
        AZ_Assert(sceneInterface, "No body found for previously given handle");

        const AZ::Transform robotTransform = rigidBody->GetTransform();

        if (m_config.m_physicalApi == RigidBodyTwistControlComponentConfig::PhysicalApi::Kinematic)
        {
            // Convert local steering to world frame
            const auto linearVelocityGlobal = robotTransform.TransformVector(m_linearVelocityLocal);
            const auto angularVelocityGlobal = robotTransform.TransformVector(m_angularVelocityLocal);

            // Set the kinematic target for the rigid body
            // This will move the rigid body to the target position and orientation
            Physics::RigidBodyRequestBus::Event(GetEntityId(), &Physics::RigidBodyRequests::SetKinematic, true);

            AZ::Transform kinematicTarget = robotTransform;
            kinematicTarget.SetTranslation(kinematicTarget.GetTranslation() + linearVelocityGlobal * fixedDeltaTime);
            kinematicTarget.SetRotation(
                AZ::Quaternion::CreateFromScaledAxisAngle(angularVelocityGlobal * fixedDeltaTime) * kinematicTarget.GetRotation());

            Physics::RigidBodyRequestBus::Event(GetEntityId(), &Physics::RigidBodyRequests::SetKinematicTarget, kinematicTarget);
            return;
        }
        if (m_config.m_physicalApi == RigidBodyTwistControlComponentConfig::PhysicalApi::Velocity)
        {
            // Convert local steering to world frame
            const auto linearVelocityGlobal = robotTransform.TransformVector(m_linearVelocityLocal);
            const auto angularVelocityGlobal = robotTransform.TransformVector(m_angularVelocityLocal);
            Physics::RigidBodyRequestBus::Event(GetEntityId(), &Physics::RigidBodyRequests::SetLinearVelocity, linearVelocityGlobal);
            Physics::RigidBodyRequestBus::Event(GetEntityId(), &Physics::RigidBodyRequests::SetAngularVelocity, angularVelocityGlobal);
            return;
        }

        if (m_config.m_physicalApi == RigidBodyTwistControlComponentConfig::PhysicalApi::Force)
        {
            const AZ::Transform robotTransformInv = rigidBody->GetTransform().GetInverse();
            AZ::Vector3 currentLinearVelocityGlob = AZ::Vector3::CreateZero();
            AZ::Vector3 currentAngularVelocityGlob = AZ::Vector3::CreateZero();
            Physics::RigidBodyRequestBus::EventResult(
                currentLinearVelocityGlob, GetEntityId(), &Physics::RigidBodyRequests::GetLinearVelocity);
            Physics::RigidBodyRequestBus::EventResult(
                currentAngularVelocityGlob, GetEntityId(), &Physics::RigidBodyRequests::GetAngularVelocity);

            // Convert local steering to world frame
            const AZ::Vector3 currentLinearVelocityLocal = robotTransformInv.TransformVector(currentLinearVelocityGlob);
            const AZ::Vector3 angularVelocityLocal = robotTransformInv.TransformVector(currentAngularVelocityGlob);

            const AZ::Vector3 errorLinear = m_linearVelocityLocal - currentLinearVelocityLocal;
            const AZ::Vector3 errorAngular = m_angularVelocityLocal - angularVelocityLocal;

            // Compute the forces to apply based on the error
            AZ::Vector3 linearForceLocal = ComputeImpulse(errorLinear, m_config.m_linerControllers, fixedDeltaTime);
            AZ::Vector3 angularForceLocal = ComputeImpulse(errorAngular, m_config.m_angularControllers, fixedDeltaTime);

            const AZ::Vector3 linearForceGlobal = robotTransform.TransformVector(linearForceLocal);
            const AZ::Vector3 angularForceGlobal = robotTransform.TransformVector(angularForceLocal);

            Physics::RigidBodyRequestBus::Event(
                GetEntityId(), &Physics::RigidBodyRequests::ApplyLinearImpulse, linearForceGlobal * fixedDeltaTime);
            Physics::RigidBodyRequestBus::Event(
                GetEntityId(), &Physics::RigidBodyRequests::ApplyAngularImpulse, angularForceGlobal * fixedDeltaTime);
        }
    }

    void RigidBodyTwistControlComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("ROS2RobotControl"));
        required.push_back(AZ_CRC_CE("PhysicsRigidBodyService"));
    }

    void RigidBodyTwistControlComponent::TwistReceived(const AZ::Vector3& linear, const AZ::Vector3& angular)
    {
        m_linearVelocityLocal = linear;
        m_angularVelocityLocal = angular;
    }
} // namespace ROS2Controllers
