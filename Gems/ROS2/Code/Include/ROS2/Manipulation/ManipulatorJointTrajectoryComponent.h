/*
* Copyright (c) Contributors to the Open 3D Engine Project.
* For complete copyright and license terms please see the LICENSE at the root of this distribution.
*
* SPDX-License-Identifier: Apache-2.0 OR MIT
*
 */

#pragma once

#include <AzCore/Component/Component.h>
#include <AzCore/Component/TickBus.h>
#include <AzCore/Entity/EntityBus.h>
#include <ROS2/Manipulation/ManipulatorTrajectoryRequestBus.h>
#include <control_msgs/action/follow_joint_trajectory.hpp>

namespace ROS2
{
    //! Base class responsible for execution of command to move robotic arm (manipulator) based on set trajectory goal.
    //! Can be derived to support different implementations.
    class ManipulatorJointTrajectoryComponent
        : public AZ::Component
        , public AZ::TickBus::Handler
        , public ManipulatorTrajectoryRequestBus::Handler
    {
    public:
        ManipulatorJointTrajectoryComponent();

        // ManipulatorTrajectoryRequestBus::Handler overrides ...
        //! @see ROS2::ManipulatorTrajectoryRequestBus::StartTrajectoryGoal
        AZ::Outcome<void, AZStd::string> StartTrajectoryGoal(TrajectoryGoalPtr trajectoryGoal) override;
        //! @see ROS2::ManipulatorTrajectoryRequestBus::CancelTrajectoryGoal
        AZ::Outcome<void, AZStd::string> CancelTrajectoryGoal(TrajectoryResultPtr trajectoryResult) override;
        //! @see ROS2::ManipulatorTrajectoryRequestBus::GetGoalStatus
        ManipulatorActionStatus GetGoalStatus() override;

    private:
        //! Follow set trajectory.
        //! @param deltaTimeNs frame time step, to advance trajectory by.
        void FollowTrajectory(const uint64_t deltaTimeNs);

        vvoid MoveToNextPoint(const trajectory_msgs::msg::JointTrajectoryPoint currentTrajectoryPoint) = 0;

        AZStd::string m_followTrajectoryActionName{ "arm_controller/follow_joint_trajectory" };
        AZStd::unique_ptr<FollowJointTrajectoryActionServer> m_followTrajectoryServer;
        ManipulatorTrajectoryRequestBus::TrajectoryGoal m_trajectoryGoal;
        ManipulatorJoints m_manipulatorJoints;
        rclcpp::Time m_trajectoryExecutionStartTime;
        AZ::EntityId m_entityId;

        bool m_initialized{ false };
        bool m_trajectoryInProgress{ false };
    };
} // namespace ROS2
