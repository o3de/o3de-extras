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

namespace ROS2
{
    void RigidBodyTwistControlComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<RigidBodyTwistControlComponent, AZ::Component>()->Version(1);
            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<RigidBodyTwistControlComponent>("Rigid Body Twist Control", "Simple control through RigidBody")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->Attribute(AZ::Edit::Attributes::Icon, "Editor/Icons/Components/RigidBodyTwistControl.svg")
                    ->Attribute(AZ::Edit::Attributes::ViewportIcon, "Editor/Icons/Components/Viewport/RigidBodyTwistControl.svg");
            }
        }
    }

    void RigidBodyTwistControlComponent::Activate()
    {
        AZ::TickBus::Handler::BusConnect();
        TwistNotificationBus::Handler::BusConnect(GetEntityId());
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
    
    void RigidBodyTwistControlComponent::OnTick([[maybe_unused]]float deltaTime, [[maybe_unused]]AZ::ScriptTimePoint time)
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
                [this, sceneInterface]([[maybe_unused]] AzPhysics::SceneHandle sceneHandle, float fixedDeltaTime)
                {
                    auto* rigidBody = sceneInterface->GetSimulatedBodyFromHandle(sceneHandle, m_bodyHandle);
                    AZ_Assert(sceneInterface, "No body found for previously given handle");

                    // Convert local steering to world frame
                    const AZ::Transform robotTransform = rigidBody->GetTransform();
                    const auto linearVelocityGlobal = robotTransform.TransformVector(m_linearVelocityLocal);
                    const auto angularVelocityGlobal = robotTransform.TransformVector(m_angularVelocityLocal);
                    Physics::RigidBodyRequestBus::Event(GetEntityId(), &Physics::RigidBodyRequests::SetLinearVelocity, linearVelocityGlobal);
                    Physics::RigidBodyRequestBus::Event(GetEntityId(), &Physics::RigidBodyRequests::SetAngularVelocity, angularVelocityGlobal);
                },
                aznumeric_cast<int32_t>(AzPhysics::SceneEvents::PhysicsStartFinishSimulationPriority::Components));
        sceneInterface->RegisterSceneSimulationFinishHandler(defaultSceneHandle, m_sceneFinishSimHandler);
        AZ::TickBus::Handler::BusDisconnect();
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
} // namespace ROS2
