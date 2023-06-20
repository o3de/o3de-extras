/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root
 * of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Component/Component.h>
#include <AzFramework/Physics/Common/PhysicsEvents.h>

#include <AzCore/Component/TickBus.h>
#include <AzFramework/Physics/Common/PhysicsSimulatedBodyEvents.h>
#include <AzFramework/Physics/RigidBodyBus.h>
#include <ImGuiBus.h>
#include <LmbrCentral/Scripting/TagComponentBus.h>
#include <ROS2/Gripper/GripperRequestBus.h>

namespace ROS2
{
    //! This component implements vacuum gripper functionality.
    //! It allows to attach to gripper objects that are in inside designated trigger collider to objects that has "Grippable" tag.
    //! To use component:
    //!  - Attach component to root of robot articulation.
    //!  - Assign to m_gripperEffectorCollider EntityId of the collider that will be used as the gripper effector.
    //!  - Add tag "Grippable" to objects that can be gripped.
    class VacuumGripper
        : public AZ::Component
        , public GripperRequestBus::Handler
        , public ImGui::ImGuiUpdateListenerBus::Handler
        , public AZ::TickBus::Handler
    {
    public:
        static constexpr AZ::Crc32 GrippableTag = AZ_CRC_CE("Grippable");
        AZ_COMPONENT(VacuumGripper, "{a29eb4fa-0f6f-11ee-be56-0242ac120002}", AZ::Component);
        VacuumGripper() = default;
        ~VacuumGripper() = default;

        // AZ::Component overrides...
        void Activate() override;
        void Deactivate() override;

        static void Reflect(AZ::ReflectContext* context);

    private:
        // GripperRequestBus::Handler overrides...
        AZ::Outcome<void, AZStd::string> GripperCommand(float position, float maxEffort) override;
        AZ::Outcome<void, AZStd::string> CancelGripperCommand() override;
        float GetGripperPosition() const override;
        float GetGripperEffort() const override;
        bool IsGripperNotMoving() const override;
        bool IsGripperReachedGoal() const override;


        // AZ::TickBus::Handler overrides...
        void OnTick(float delta, AZ::ScriptTimePoint timePoint) override;

        // ImGui::ImGuiUpdateListenerBus::Handler overrides...
        void OnImGuiUpdate() override;

        //! The entity that contains collider that will be used as the gripper
        //! effector/ The collider must be a trigger collider.
        AZ::EntityId m_gripperEffectorCollider;

        //! EntityId of the object that is currently gripped by the gripper effector.
        AZ::EntityId m_grippedObjectInEffector;

        //! Handle to joint created by vacuum gripper.
        AzPhysics::JointHandle m_vaccumJoint;

        bool m_gripping {false};

        AzPhysics::SimulatedBodyEvents::OnTriggerEnter::Handler m_onTriggerEnterHandler;
        AzPhysics::SimulatedBodyEvents::OnTriggerExit::Handler m_onTriggerExitHandler;

        //! check if object is grippable (has Tag).
        bool isObjectGrippable(const AZ::EntityId entityId);

        //! check if oject is in the gripper effector collider and creates joint between gripper effector and object.
        bool TryToGripObject();

        //! releases object from gripper effector.
        void ReleaseGrippedObject();
    };
} // namespace ROS2
