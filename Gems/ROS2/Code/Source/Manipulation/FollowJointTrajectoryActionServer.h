/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/Component/EntityId.h>
#include <AzCore/std/string/string.h>
#include <ROS2/Manipulation/JointsTrajectoryRequests.h>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_action/server.hpp>

namespace ROS2
{
    //! A class wrapping ROS 2 action server for joint trajectory controller.
    //! @see <a href="https://control.ros.org/master/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html"> joint trajectory
    //! controller </a>.
    class FollowJointTrajectoryActionServer
    {
    public:
        using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;

        //! Create an action server for FollowJointTrajectory action and bind Goal callbacks.
        //! @param actionName Name of the action, similar to topic or service name.
        //! @param entityId entity which will execute callbacks through JointsTrajectoryRequestBus.
        //! @see <a href="https://docs.ros.org/en/humble/p/rclcpp_action/generated/classrclcpp__action_1_1Server.html"> ROS 2 action
        //! server documentation </a>
        FollowJointTrajectoryActionServer(const AZStd::string& actionName, const AZ::EntityId& entityId);

        //! Return trajectory action status.
        //! @return Status of the trajectory execution.
        JointsTrajectoryRequests::TrajectoryActionStatus GetGoalStatus() const;

        //! Cancel the current goal.
        //! @param result Result to be passed to through action server to the client.
        void CancelGoal(std::shared_ptr<FollowJointTrajectory::Result> result);

        //! Sets the goal status to success
        void SetGoalSuccess();

        //! Report goal success to the action server.
        //! @param result Result which contains success code.
        void GoalSuccess(std::shared_ptr<FollowJointTrajectory::Result> result);

        //! Publish feedback during an active action.
        //! @param feedback An action feedback message informing about the progress.
        void PublishFeedback(std::shared_ptr<FollowJointTrajectory::Feedback> feedback);

    private:
        using GoalHandle = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;
        using TrajectoryActionStatus = JointsTrajectoryRequests::TrajectoryActionStatus;

        AZ::EntityId m_entityId;
        TrajectoryActionStatus m_goalStatus = TrajectoryActionStatus::Idle;
        rclcpp_action::Server<FollowJointTrajectory>::SharedPtr m_actionServer;
        std::shared_ptr<GoalHandle> m_goalHandle;

        bool IsGoalActiveState() const;
        bool IsReadyForExecution() const;
        bool IsExecuting() const;

        rclcpp_action::GoalResponse GoalReceivedCallback(
            const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const FollowJointTrajectory::Goal> goal);

        rclcpp_action::CancelResponse GoalCancelledCallback(const std::shared_ptr<GoalHandle> goalHandle);

        void GoalAcceptedCallback(const std::shared_ptr<GoalHandle> goalHandle);
    };
} // namespace ROS2
