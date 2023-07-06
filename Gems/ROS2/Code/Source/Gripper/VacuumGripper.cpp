/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root
 * of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "VacuumGripper.h"
#include "Utils.h"

#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzFramework/Components/TransformComponent.h>

#include "Source/ArticulationLinkComponent.h"
#include <AzFramework/Physics/PhysicsSystem.h>
#include <AzFramework/Physics/RigidBodyBus.h>
#include <AzFramework/Physics/SimulatedBodies/RigidBody.h>
#include <imgui/imgui.h>

namespace ROS2
{
    void VacuumGripper::Activate()
    {
        m_gripperEffectorBodyHandle = AzPhysics::InvalidSimulatedBodyHandle;
        m_onTriggerEnterHandler = AzPhysics::SimulatedBodyEvents::OnTriggerEnter::Handler(
            [&]([[maybe_unused]] AzPhysics::SimulatedBodyHandle bodyHandle, [[maybe_unused]] const AzPhysics::TriggerEvent& event)
            {
                AZ_Printf("ROS2", "VacuumGripper::Activate() - OnTriggerEnter\n");
                const auto boxId = event.m_otherBody->GetEntityId();
                const bool isGrippable = isObjectGrippable(boxId);
                if (isGrippable)
                {
                    m_grippedObjectInEffector = boxId;
                }
            });

        m_onTriggerExitHandler = AzPhysics::SimulatedBodyEvents::OnTriggerExit::Handler(
            [&]([[maybe_unused]] AzPhysics::SimulatedBodyHandle bodyHandle, [[maybe_unused]] const AzPhysics::TriggerEvent& event)
            {
                AZ_Printf("ROS2", "VacuumGripper::Activate() - OnTriggerExit\n");
                const auto boxId = event.m_otherBody->GetEntityId();
                const bool isGrippable = isObjectGrippable(boxId);
                if (isGrippable)
                {
                    m_grippedObjectInEffector = AZ::EntityId(AZ::EntityId::InvalidEntityId);
                }
            });
        m_vaccumJoint = AzPhysics::InvalidJointHandle;
        m_grippedObjectInEffector = AZ::EntityId(AZ::EntityId::InvalidEntityId);
        m_gripping = false;

        AZ::TickBus::Handler::BusConnect();
        ImGui::ImGuiUpdateListenerBus::Handler::BusConnect();
        GripperRequestBus::Handler::BusConnect(GetEntityId());
    }

    void VacuumGripper::Deactivate()
    {
        m_onTriggerEnterHandler.Disconnect();
        m_onTriggerExitHandler.Disconnect();
        GripperRequestBus::Handler::BusDisconnect();
        AZ::TickBus::Handler::BusDisconnect();
        ImGui::ImGuiUpdateListenerBus::Handler::BusDisconnect();
    }

    void VacuumGripper::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<VacuumGripper, AZ::Component>()
                ->Field("EffectorCollider", &VacuumGripper::m_gripperEffectorCollider)
                ->Field("EffectorArticulation", &VacuumGripper::m_gripperEffectorArticulationLink)
                ->Version(1);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<VacuumGripper>("VacuumGripper", "VacuumGripper")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "VacuumGripper")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->DataElement(
                        AZ::Edit::UIHandlers::EntityId,
                        &VacuumGripper::m_gripperEffectorCollider,
                        "Effector Trigger Collider",
                        "The entity with trigger collider that will detect objects that can be successfully gripped.")
                    ->DataElement(
                        AZ::Edit::UIHandlers::EntityId,
                        &VacuumGripper::m_gripperEffectorArticulationLink,
                        "Effector Articulation Link",
                        "The entity that is the articulation link of the effector.");
            }
        }
    }

    void VacuumGripper::OnTick(float delta, AZ::ScriptTimePoint timePoint)
    {
        AzPhysics::SystemInterface* physicsSystem = AZ::Interface<AzPhysics::SystemInterface>::Get();
        AZ_Assert(physicsSystem, "No physics system.");

        AzPhysics::SceneInterface* sceneInterface = AZ::Interface<AzPhysics::SceneInterface>::Get();
        AZ_Assert(sceneInterface, "No scene intreface.");

        AzPhysics::SceneHandle defaultSceneHandle = sceneInterface->GetSceneHandle(AzPhysics::DefaultPhysicsSceneName);
        AZ_Assert(defaultSceneHandle != AzPhysics::InvalidSceneHandle, "Invalid default physics scene handle.");

        // Connect the trigger handlers if not already connected, it is circle navigation around issue GH-16188, the
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
            AZ_Printf("VacuumGripper", "Registered trigger handlers.");

            AZ::EntityId rootArticulationEntity = GetRootOfArticulation(m_gripperEffectorArticulationLink);
            AZ::Entity* rootEntity = nullptr;
            AZ::ComponentApplicationBus::BroadcastResult(rootEntity, &AZ::ComponentApplicationRequests::FindEntity, rootArticulationEntity);

            AZ_Printf("VacuumGripper", "Root articulation entity name: %s", rootEntity->GetName().c_str());

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
        if (m_gripping)
        {
            TryToGripObject();
        }
    }
    AZ::EntityId VacuumGripper::GetRootOfArticulation(AZ::EntityId entityId)
    {
        AZ::EntityId parentEntityId{ AZ::EntityId::InvalidEntityId };
        AZ::Entity* parentEntity = nullptr;
        AZ::TransformBus::EventResult(parentEntityId, entityId, &AZ::TransformBus::Events::GetParentId);
        AZ::ComponentApplicationBus::BroadcastResult(parentEntity, &AZ::ComponentApplicationRequests::FindEntity, parentEntityId);

        if (parentEntity == nullptr)
        {
            return AZ::EntityId(AZ::EntityId::InvalidEntityId);
        }

        // get articulation link component, if not found for parent, return current entity
        PhysX::ArticulationLinkComponent* component = parentEntity->FindComponent<PhysX::ArticulationLinkComponent>();
        if (component == nullptr)
        {
            return entityId;
        }
        return GetRootOfArticulation(parentEntity->GetId());
    }

    bool VacuumGripper::isObjectGrippable(const AZ::EntityId entityId)
    {
        bool isGrippable = false;
        LmbrCentral::TagComponentRequestBus::EventResult(isGrippable, entityId, &LmbrCentral::TagComponentRequests::HasTag, GrippableTag);
        return isGrippable;
    }

    bool VacuumGripper::TryToGripObject()
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
        if (m_vaccumJoint != AzPhysics::InvalidJointHandle)
        {
            // Object is already gripped
            return true;
        }

        if (!m_grippedObjectInEffector.IsValid())
        {
            // No object to grip
            return false;
        }
        PhysX::ArticulationLinkComponent* component = m_entity->FindComponent<PhysX::ArticulationLinkComponent>();
        AZ_Assert(component, "No PhysX::ArticulationLinkComponent found on entity ");

        auto* sceneInterface = AZ::Interface<AzPhysics::SceneInterface>::Get();
        AzPhysics::SceneHandle defaultSceneHandle = sceneInterface->GetSceneHandle(AzPhysics::DefaultPhysicsSceneName);
        AZ_Assert(defaultSceneHandle != AzPhysics::InvalidSceneHandle, "Invalid default physics scene handle");

        // Get gripped rigid body
        AzPhysics::RigidBody* grippedRigidBody = nullptr;
        Physics::RigidBodyRequestBus::EventResult(grippedRigidBody, m_grippedObjectInEffector, &Physics::RigidBodyRequests::GetRigidBody);
        AZ_Assert(grippedRigidBody, "No RigidBody found on entity grippedRigidBody");

        // Gripper is end of articulation chain
        AzPhysics::SimulatedBody* gripperBody = nullptr;
        gripperBody = sceneInterface->GetSimulatedBodyFromHandle(defaultSceneHandle, m_gripperEffectorBodyHandle);
        AZ_Assert(gripperBody, "No gripper body found");

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

        // create new joint
        m_vaccumJoint =
            sceneInterface->AddJoint(defaultSceneHandle, &jointConfig, m_gripperEffectorBodyHandle, grippedRigidBody->m_bodyHandle);

        return true;
    }

    void VacuumGripper::ReleaseGrippedObject()
    {
        m_gripping = false;
        if (m_vaccumJoint == AzPhysics::InvalidJointHandle)
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
        sceneInterface->RemoveJoint(defaultSceneHandle, m_vaccumJoint);
        m_vaccumJoint = AzPhysics::InvalidJointHandle;
    }

    void VacuumGripper::OnImGuiUpdate()
    {
        ImGui::Begin("VacuumGripperDebugger");

        AZStd::string grippedObjectInEffectorName;
        AZ::ComponentApplicationBus::BroadcastResult(
            grippedObjectInEffectorName, &AZ::ComponentApplicationRequests::GetEntityName, m_grippedObjectInEffector);

        ImGui::Text("Grippable object : %s", grippedObjectInEffectorName.c_str());
        ImGui::Text("Vacuum joint created : %d ", m_vaccumJoint != AzPhysics::InvalidJointHandle);

        ImGui::Checkbox("Gripping", &m_gripping);

        if (ImGui::Button("Grip Command "))
        {
            m_gripping = true;
        }

        if (ImGui::Button("Release Command"))
        {
            ReleaseGrippedObject();
        }
        ImGui::End();
    }

    AZ::Outcome<void, AZStd::string> VacuumGripper::GripperCommand(float position, float maxEffort)
    {
        AZ_Printf("VacuumGripper", "GripperCommand %f", position);
        if (position > 0)
        {
            m_gripping = true;
        }
        else
        {
            ReleaseGrippedObject();
        }
        return AZ::Success();
    }

    AZ::Outcome<void, AZStd::string> VacuumGripper::CancelGripperCommand()
    {
        ReleaseGrippedObject();
        return AZ::Success();
    }

    float VacuumGripper::GetGripperPosition() const
    {
        return m_vaccumJoint == AzPhysics::InvalidJointHandle ? 0.0f : 1.0f;
    }

    float VacuumGripper::GetGripperEffort() const
    {
        return m_vaccumJoint == AzPhysics::InvalidJointHandle ? 0.0f : 1.0f;
    }
    bool VacuumGripper::IsGripperNotMoving() const
    {
        return true;
    }

    bool VacuumGripper::HasGripperReachedGoal() const
    {
        const bool isJoint = (m_vaccumJoint != AzPhysics::InvalidJointHandle);
        if (m_gripping && isJoint)
        {
            return true;
        }
        if (!m_gripping && !isJoint)
        {
            return true;
        }
        return false;
    }

} // namespace ROS2
