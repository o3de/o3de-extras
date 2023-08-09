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
#include <AzCore/Component/TickBus.h>
#include <AzFramework/Physics/Common/PhysicsEvents.h>
#include <ImGuiBus.h>
#include <ROS2/Gripper/GripperRequestBus.h>
#include <ROS2/Manipulation/JointsManipulationRequests.h>
#include <Utilities/ArticulationsUtilities.h>

namespace ROS2
{
    //! This component implements finger gripper functionality.
    class FingerGripperComponent
        : public AZ::Component
        , public GripperRequestBus::Handler
        , public ImGui::ImGuiUpdateListenerBus::Handler
        , public AZ::TickBus::Handler
    {
    public:
        AZ_COMPONENT(FingerGripperComponent, "{ae5f8ec2-26ee-11ee-be56-0242ac120002}", AZ::Component);
        FingerGripperComponent() = default;
        ~FingerGripperComponent() = default;

        // AZ::Component overrides...
        void Activate() override;
        void Deactivate() override;

        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);

        static void Reflect(AZ::ReflectContext* context);

    private:
        // GripperRequestBus::Handler overrides...
        AZ::Outcome<void, AZStd::string> GripperCommand(float position, float maxEffort) override;
        AZ::Outcome<void, AZStd::string> CancelGripperCommand() override;
        bool HasGripperCommandBeenCancelled() const override;

        // Sum of all joint positions
        float GetGripperPosition() const override;
        // Sum of all efforts exerted by fingers
        float GetGripperEffort() const override;
        // Non-articulation fingers return 0 effort!
        bool IsGripperNotMoving() const override;
        // Doesn't check if the max force is applied, only checks speed
        bool HasGripperReachedGoal() const override;

        // ImGui::ImGuiUpdateListenerBus::Handler overrides...
        void OnImGuiUpdate() override;

        // AZ::TickBus::Handler overrides...
        void OnTick(float delta, AZ::ScriptTimePoint timePoint) override;

        ManipulationJoints& GetFingerJoints();

        AZ::EntityId m_rootOfArticulation; //!< The root of the articulation chain

        float GetDefaultPosition();
        void SetPosition(float position, float maxEffort);
        bool IsGripperVelocity0() const;
        void PublishFeedback() const;

        ManipulationJoints m_fingerJoints;
        bool m_grippingInProgress{ false };
        bool m_cancelled{ false };
        bool m_initialised{ false };
        float m_desiredPosition{ false };
        float m_stallingFor{ 0.f };
        float m_ImGuiPosition{ 0.1f };

        float m_velocityEpsilon{ 0.01f }; //!< The epsilon value used to determine whether the gripper is moving
        float m_goalTolerance{ 0.001f }; //!< The epsilon value used to determine whether the gripper reached it's goal
        float m_stallTime{ 0.1f }; //!< The time in seconds to wait before determining the gripper is stalled
    };
} // namespace ROS2
