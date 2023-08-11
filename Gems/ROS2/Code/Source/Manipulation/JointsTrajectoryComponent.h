/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include "FollowJointTrajectoryActionServer.h"
#include <AzCore/Component/Component.h>
#include <AzCore/Component/EntityBus.h>
#include <AzCore/Component/TickBus.h>
#include <ROS2/Manipulation/JointsManipulationRequests.h>
#include <ROS2/Manipulation/JointsTrajectoryRequests.h>
#include <control_msgs/action/follow_joint_trajectory.hpp>

namespace ROS2
{
    //! Component responsible for execution of commands to move robotic arm (manipulator) based on set trajectory goal.
    class JointsTrajectoryComponent
        : public AZ::Component
        , public AZ::TickBus::Handler
        , public JointsTrajectoryRequestBus::Handler
    {
    public:
        JointsTrajectoryComponent() = default;
        ~JointsTrajectoryComponent() = default;
        AZ_COMPONENT(JointsTrajectoryComponent, "{429DE04C-6B6D-4B2D-9D6C-3681F23CBF90}", AZ::Component);

        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void Reflect(AZ::ReflectContext* context);

        // JointsTrajectoryRequestBus::Handler overrides ...
        //! @see ROS2::JointsTrajectoryRequestBus::StartTrajectoryGoal
        AZ::Outcome<void, TrajectoryResult> StartTrajectoryGoal(TrajectoryGoalPtr trajectoryGoal) override;
        //! @see ROS2::JointsTrajectoryRequestBus::CancelTrajectoryGoal
        AZ::Outcome<void, AZStd::string> CancelTrajectoryGoal() override;
        //! @see ROS2::JointsTrajectoryRequestBus::GetGoalStatus
        TrajectoryActionStatus GetGoalStatus() override;

    private:
        // Component overrides ...
        void Activate() override;
        void Deactivate() override;

        // AZ::TickBus::Handler overrides
        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;

        //! Follow set trajectory.
        //! @param deltaTimeNs frame time step, to advance trajectory by.
        void FollowTrajectory(const uint64_t deltaTimeNs);
        AZ::Outcome<void, TrajectoryResult> ValidateGoal(TrajectoryGoalPtr trajectoryGoal);
        void MoveToNextPoint(const trajectory_msgs::msg::JointTrajectoryPoint currentTrajectoryPoint);
        void UpdateFeedback();

        //! Lazy initialize Manipulation joints on the start of simulation.
        ManipulationJoints& GetManipulationJoints();

        AZStd::string m_followTrajectoryActionName{ "arm_controller/follow_joint_trajectory" };
        AZStd::unique_ptr<FollowJointTrajectoryActionServer> m_followTrajectoryServer;
        TrajectoryGoal m_trajectoryGoal;
        rclcpp::Time m_trajectoryExecutionStartTime;
        ManipulationJoints m_manipulationJoints;
        bool m_trajectoryInProgress{ false };
    };
} // namespace ROS2
