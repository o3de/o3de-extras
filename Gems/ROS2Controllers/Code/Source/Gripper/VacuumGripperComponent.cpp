/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root
 * of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "VacuumGripperComponent.h"
#include "Source/ArticulationLinkComponent.h"
#include "Utils.h"
#include <Utilities/ArticulationsUtilities.h>

#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzFramework/Components/TransformComponent.h>
#include <AzFramework/Physics/PhysicsSystem.h>
#include <AzFramework/Physics/RigidBodyBus.h>
#include <AzFramework/Physics/SimulatedBodies/RigidBody.h>
#include <imgui/imgui.h>

namespace ROS2
{
    void VacuumGripperComponent::Activate()
    {
        m_gripperEffectorBodyHandle = AzPhysics::InvalidSimulatedBodyHandle;
        m_onTriggerEnterHandler = AzPhysics::SimulatedBodyEvents::OnTriggerEnter::Handler(
            [&]([[maybe_unused]] AzPhysics::SimulatedBodyHandle bodyHandle, [[maybe_unused]] const AzPhysics::TriggerEvent& event)
            {
                const auto grippedEntityCandidateId = event.m_otherBody->GetEntityId();
                const bool isGrippable = isObjectGrippable(grippedEntityCandidateId);
                if (isGrippable)
                {
                    m_grippedObjectInEffector = grippedEntityCandidateId;
                }
            });

        m_onTriggerExitHandler = AzPhysics::SimulatedBodyEvents::OnTriggerExit::Handler(
            [&]([[maybe_unused]] AzPhysics::SimulatedBodyHandle bodyHandle, [[maybe_unused]] const AzPhysics::TriggerEvent& event)
            {
                const auto grippedEntityCandidateId = event.m_otherBody->GetEntityId();
                if (m_grippedObjectInEffector == grippedEntityCandidateId)
                {
                    m_grippedObjectInEffector = AZ::EntityId(AZ::EntityId::InvalidEntityId);
                }
            });

        m_vacuumJoint = AzPhysics::InvalidJointHandle;
        m_grippedObjectInEffector = AZ::EntityId(AZ::EntityId::InvalidEntityId);
        m_tryingToGrip = false;
        m_cancelGripperCommand = false;
        AZ::TickBus::Handler::BusConnect();
        ImGui::ImGuiUpdateListenerBus::Handler::BusConnect();
        GripperRequestBus::Handler::BusConnect(GetEntityId());
    }

    void VacuumGripperComponent::Deactivate()
    {
        m_onTriggerEnterHandler.Disconnect();
        m_onTriggerExitHandler.Disconnect();
        GripperRequestBus::Handler::BusDisconnect();
        AZ::TickBus::Handler::BusDisconnect();
        ImGui::ImGuiUpdateListenerBus::Handler::BusDisconnect();
    }

    void VacuumGripperComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("GripperService"));
    }
    void VacuumGripperComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("GripperService"));
    }

    void VacuumGripperComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<VacuumGripperComponent, AZ::Component>()
                ->Field("EffectorCollider", &VacuumGripperComponent::m_gripperEffectorCollider)
                ->Field("EffectorArticulation", &VacuumGripperComponent::m_gripperEffectorArticulationLink)
                ->Version(1);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<VacuumGripperComponent>("VacuumGripperComponent", "Component for control of a vacuum gripper.")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "VacuumGripper")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->Attribute(AZ::Edit::Attributes::Icon, "Editor/Icons/Components/VacuumGripperComponent.svg")
                    ->Attribute(AZ::Edit::Attributes::ViewportIcon, "Editor/Icons/Components/Viewport/VacuumGripperComponent.svg")
                    ->DataElement(
                        AZ::Edit::UIHandlers::EntityId,
                        &VacuumGripperComponent::m_gripperEffectorCollider,
                        "Effector Trigger Collider",
                        "The entity with trigger collider that will detect objects that can be successfully gripped.")
                    ->DataElement(
                        AZ::Edit::UIHandlers::EntityId,
                        &VacuumGripperComponent::m_gripperEffectorArticulationLink,
                        "Effector Articulation Link",
                        "The entity that is the articulation link of the effector.");
            }
        }
    }

    void VacuumGripperComponent::OnTick(float delta, AZ::ScriptTimePoint timePoint)
    {
        AZ_Assert(AZ::Interface<AzPhysics::SystemInterface>::Get(), "No physics system.");

        AzPhysics::SceneInterface* sceneInterface = AZ::Interface<AzPhysics::SceneInterface>::Get();
        AZ_Assert(sceneInterface, "No scene intreface.");

        AzPhysics::SceneHandle defaultSceneHandle = sceneInterface->GetSceneHandle(AzPhysics::DefaultPhysicsSceneName);
        AZ_Assert(defaultSceneHandle != AzPhysics::InvalidSceneHandle, "Invalid default physics scene handle.");

        // Connect the trigger handlers if not already connected, it is circumventing the issue GH-16188, the
        // RigidbodyNotificationBus should be used instead.
        if (!m_onTriggerEnterHandler.IsConnected() || !m_onTriggerExitHandler.IsConnected())
        {
            if (auto* physicsSystem = AZ::Interface<AzPhysics::SystemInterface>::Get())
            {
                AZStd::pair<AzPhysics::SceneHandle, AzPhysics::SimulatedBodyHandle> foundBody =
                    physicsSystem->FindAttachedBodyHandleFromEntityId(m_gripperEffectorCollider);
                AZ_Warning(
                    "VacuumGripper", foundBody.first != AzPhysics::InvalidSceneHandle, "No body found for m_gripperEffectorCollider.");
                if (foundBody.first != AzPhysics::InvalidSceneHandle)
                {
                    AzPhysics::SimulatedBodyEvents::RegisterOnTriggerEnterHandler(
                        foundBody.first, foundBody.second, m_onTriggerEnterHandler);
                    AzPhysics::SimulatedBodyEvents::RegisterOnTriggerExitHandler(foundBody.first, foundBody.second, m_onTriggerExitHandler);
                }
            }

            AZ::EntityId rootArticulationEntity = Utils::GetRootOfArticulation(m_gripperEffectorArticulationLink);
            AZ::Entity* rootEntity = nullptr;
            AZ::ComponentApplicationBus::BroadcastResult(rootEntity, &AZ::ComponentApplicationRequests::FindEntity, rootArticulationEntity);

            AZ_Trace("VacuumGripper", "Root articulation entity name: %s\n", rootEntity->GetName().c_str());

            PhysX::ArticulationLinkComponent* component = rootEntity->FindComponent<PhysX::ArticulationLinkComponent>();
            AZStd::vector<AzPhysics::SimulatedBodyHandle> articulationHandles = component->GetSimulatedBodyHandles();

            AZ_Assert(articulationHandles.size() > 1, "Expected more than one body handles in articulations");
            for (AzPhysics::SimulatedBodyHandle handle : articulationHandles)
            {
                AzPhysics::SimulatedBody* body = sceneInterface->GetSimulatedBodyFromHandle(defaultSceneHandle, handle);
                AZ_Assert(body, "Expected valid body pointer");
                if (body->GetEntityId() == m_gripperEffectorArticulationLink)
                {
                    m_gripperEffectorBodyHandle = handle;
                }
            }
        }
        if (m_tryingToGrip)
        {
            TryToGripObject();
        }
    }

    bool VacuumGripperComponent::isObjectGrippable(const AZ::EntityId entityId)
    {
        bool isGrippable = false;
        LmbrCentral::TagComponentRequestBus::EventResult(isGrippable, entityId, &LmbrCentral::TagComponentRequests::HasTag, GrippableTag);
        return isGrippable;
    }

    bool VacuumGripperComponent::TryToGripObject()
    {
        AZ_Warning(
            "VacuumGripper",
            m_gripperEffectorBodyHandle != AzPhysics::InvalidSimulatedBodyHandle,
            "Invalid body handle for gripperEffectorBody");
        if (m_gripperEffectorBodyHandle == AzPhysics::InvalidSimulatedBodyHandle)
        {
            // No articulation link found
            return true;
        }
        if (m_vacuumJoint != AzPhysics::InvalidJointHandle)
        {
            // Object is already gripped
            return true;
        }

        if (!m_grippedObjectInEffector.IsValid())
        {
            // No object to grip
            return false;
        }
        AZ_Assert(m_entity->FindComponent<PhysX::ArticulationLinkComponent>(), "No PhysX::ArticulationLinkComponent found on entity ");

        auto* sceneInterface = AZ::Interface<AzPhysics::SceneInterface>::Get();
        AzPhysics::SceneHandle defaultSceneHandle = sceneInterface->GetSceneHandle(AzPhysics::DefaultPhysicsSceneName);
        AZ_Assert(defaultSceneHandle != AzPhysics::InvalidSceneHandle, "Invalid default physics scene handle");

        // Get gripped rigid body
        AzPhysics::RigidBody* grippedRigidBody = nullptr;
        Physics::RigidBodyRequestBus::EventResult(grippedRigidBody, m_grippedObjectInEffector, &Physics::RigidBodyRequests::GetRigidBody);
        AZ_Assert(grippedRigidBody, "No RigidBody found on entity grippedRigidBody");

        // Gripper is the end of the articulation chain
        AzPhysics::SimulatedBody* gripperBody = nullptr;
        gripperBody = sceneInterface->GetSimulatedBodyFromHandle(defaultSceneHandle, m_gripperEffectorBodyHandle);
        AZ_Assert(gripperBody, "No gripper body found");

        AttachToGripper(gripperBody, grippedRigidBody, sceneInterface);

        return true;
    }

    void VacuumGripperComponent::AttachToGripper(
        AzPhysics::SimulatedBody* gripperBody, AzPhysics::RigidBody* grippedRigidBody, AzPhysics::SceneInterface* sceneInterface)
    {
        AzPhysics::SceneHandle defaultSceneHandle = sceneInterface->GetSceneHandle(AzPhysics::DefaultPhysicsSceneName);
        AZ_Assert(defaultSceneHandle != AzPhysics::InvalidSceneHandle, "Invalid default physics scene handle");

        // Find Transform of the child in parent's frame
        AZ::Transform childTransformWorld = grippedRigidBody->GetTransform();
        AZ::Transform parentsTranformWorld = gripperBody->GetTransform();
        AZ::Transform childTransformParent = parentsTranformWorld.GetInverse() * childTransformWorld;
        childTransformParent.Invert();

        // Configure new joint
        PhysX::FixedJointConfiguration jointConfig;
        jointConfig.m_debugName = "VacuumJoint";
        jointConfig.m_parentLocalRotation = AZ::Quaternion::CreateIdentity();
        jointConfig.m_parentLocalPosition = AZ::Vector3::CreateZero();
        jointConfig.m_childLocalRotation = childTransformParent.GetRotation();
        jointConfig.m_childLocalPosition = childTransformParent.GetTranslation();
        jointConfig.m_startSimulationEnabled = true;

        // Create new joint
        m_vacuumJoint =
            sceneInterface->AddJoint(defaultSceneHandle, &jointConfig, m_gripperEffectorBodyHandle, grippedRigidBody->m_bodyHandle);
    }

    void VacuumGripperComponent::ReleaseGrippedObject()
    {
        m_tryingToGrip = false;
        if (m_vacuumJoint == AzPhysics::InvalidJointHandle)
        {
            return;
        }
        auto* sceneInterface = AZ::Interface<AzPhysics::SceneInterface>::Get();
        AzPhysics::SceneHandle defaultSceneHandle = sceneInterface->GetSceneHandle(AzPhysics::DefaultPhysicsSceneName);
        AZ_Assert(defaultSceneHandle != AzPhysics::InvalidSceneHandle, "Invalid default physics scene handle");
        // Wake up the body to prevent it from not moving after release
        AzPhysics::RigidBody* grippedRigidBody = nullptr;
        Physics::RigidBodyRequestBus::EventResult(grippedRigidBody, m_grippedObjectInEffector, &Physics::RigidBodyRequests::GetRigidBody);
        if (grippedRigidBody)
        {
            grippedRigidBody->ForceAwake();
        }
        sceneInterface->RemoveJoint(defaultSceneHandle, m_vacuumJoint);
        m_vacuumJoint = AzPhysics::InvalidJointHandle;
    }

    void VacuumGripperComponent::OnImGuiUpdate()
    {
        ImGui::Begin("VacuumGripperDebugger");

        AZStd::string grippedObjectInEffectorName;
        AZ::ComponentApplicationBus::BroadcastResult(
            grippedObjectInEffectorName, &AZ::ComponentApplicationRequests::GetEntityName, m_grippedObjectInEffector);

        ImGui::Text("Grippable object : %s", grippedObjectInEffectorName.c_str());
        ImGui::Text("Vacuum joint created : %d ", m_vacuumJoint != AzPhysics::InvalidJointHandle);

        ImGui::Checkbox("Gripping", &m_tryingToGrip);

        if (ImGui::Button("Grip Command "))
        {
            m_tryingToGrip = true;
        }

        if (ImGui::Button("Release Command"))
        {
            ReleaseGrippedObject();
        }
        ImGui::End();
    }

    AZ::Outcome<void, AZStd::string> VacuumGripperComponent::GripperCommand(float position, float maxEffort)
    {
        AZ_Trace("VacuumGripper", "GripperCommand %f\n", position);
        m_cancelGripperCommand = false;
        if (position == 0.0f)
        {
            m_tryingToGrip = true;
        }
        else
        {
            ReleaseGrippedObject();
        }
        return AZ::Success();
    }

    AZ::Outcome<void, AZStd::string> VacuumGripperComponent::CancelGripperCommand()
    {
        ReleaseGrippedObject();
        m_cancelGripperCommand = true;
        return AZ::Success();
    }

    bool VacuumGripperComponent::HasGripperCommandBeenCancelled() const
    {
        if (m_cancelGripperCommand && m_vacuumJoint == AzPhysics::InvalidJointHandle)
        {
            return true;
        }
        return false;
    }

    float VacuumGripperComponent::GetGripperPosition() const
    {
        return m_tryingToGrip ? 0.0f : 1.0f;
    }

    float VacuumGripperComponent::GetGripperEffort() const
    {
        return m_vacuumJoint == AzPhysics::InvalidJointHandle ? 0.0f : 1.0f;
    }
    bool VacuumGripperComponent::IsGripperNotMoving() const
    {
        return true;
    }

    bool VacuumGripperComponent::HasGripperReachedGoal() const
    {
        const bool isObjectAttached = (m_vacuumJoint != AzPhysics::InvalidJointHandle);
        if (m_tryingToGrip && isObjectAttached)
        {
            return true;
        }
        if (!m_tryingToGrip && !isObjectAttached)
        {
            return true;
        }
        return false;
    }

} // namespace ROS2
