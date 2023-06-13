/*
* Copyright (c) Contributors to the Open 3D Engine Project.
* For complete copyright and license terms please see the LICENSE at the root of this distribution.
*
* SPDX-License-Identifier: Apache-2.0 OR MIT
*
*/

#pragma once

#include <AzCore/std/string/string.h>
#include <AzCore/std/functional.h>
#include <rclcpp_action/server.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <ROS2/ROS2Bus.h>

namespace ROS2
{
    enum class GoalStatus
    {
        Pending,
        Active,
        Concluded
    };

    // ROS2 Action Server class
    class FollowJointTrajectoryActionServer
    {
    public:
        using GoalHandleFollowJointTrajectory = rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>;
        using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
        FollowJointTrajectoryActionServer() = default;
        ~FollowJointTrajectoryActionServer() = default;
        void CreateServer(AZStd::string ROS2ControllerName);

        rclcpp_action::Server<FollowJointTrajectory>::SharedPtr m_actionServer;
        std::shared_ptr<GoalHandleFollowJointTrajectory> m_goalHandle;

        GoalStatus m_goalStatus = GoalStatus::Pending;

    protected:
        // callbacks for action_server_
        rclcpp_action::GoalResponse goal_received_callback(
            const rclcpp_action::GoalUUID & uuid, 
            std::shared_ptr<const FollowJointTrajectory::Goal> goal);
        rclcpp_action::CancelResponse goal_cancelled_callback(
            const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle);
        void goal_accepted_callback(
            std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle);
    };

    void FollowJointTrajectoryActionServer::CreateServer(AZStd::string ROS2ControllerName)
    {
        auto ros2Node = ROS2Interface::Get()->GetNode();
        // Create the ROS2 action server
        this->m_actionServer = rclcpp_action::create_server<FollowJointTrajectory>(
            ros2Node,
            ROS2ControllerName.append("/follow_joint_trajectory").data(),
            AZStd::bind(&FollowJointTrajectoryActionServer::goal_received_callback, this, AZStd::placeholders::_1, AZStd::placeholders::_2),
            AZStd::bind(&FollowJointTrajectoryActionServer::goal_cancelled_callback, this, AZStd::placeholders::_1),
            AZStd::bind(&FollowJointTrajectoryActionServer::goal_accepted_callback, this, AZStd::placeholders::_1));
    }

    rclcpp_action::GoalResponse FollowJointTrajectoryActionServer::goal_received_callback(
            [[maybe_unused]] const rclcpp_action::GoalUUID & uuid,
            [[maybe_unused]] std::shared_ptr<const FollowJointTrajectory::Goal> goal)
    {
        // Dummy implementation
        AZ_TracePrintf("ManipulatorControllerComponent", "FollowJointTrajectory manipulator Goal received");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse FollowJointTrajectoryActionServer::goal_cancelled_callback(
            [[maybe_unused]] const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle)
    {
        // Dummy implementation
        AZ_TracePrintf("ManipulatorControllerComponent", "FollowJointTrajectory manipulator Goal canceled");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void FollowJointTrajectoryActionServer::goal_accepted_callback(
            const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle)
    {
        AZ_TracePrintf("ManipulatorControllerComponent", "FollowJointTrajectory manipulator Goal accepted");
        this->m_goalHandle = goal_handle;
        this->m_goalStatus = GoalStatus::Active;     
    }
}