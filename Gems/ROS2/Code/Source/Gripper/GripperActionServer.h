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
#include <ROS2/Gripper/GripperRequestBus.h>
#include <control_msgs/action/gripper_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

namespace ROS2
{
    //! GripperActionServer is a class responsible for managing an action server controlling a gripper
    //! @see <a href="https://docs.ros.org/en/humble/p/rclcpp_action/generated/classrclcpp__action_1_1Server.html"> ROS 2 action
    //! server documentation </a>
    //! @see <a href="https://docs.ros.org/en/api/control_msgs/html/msg/GripperCommand.html"> GripperCommand documentation </a>
    class GripperActionServer
    {
    public:
        using GripperCommand = control_msgs::action::GripperCommand;
        using GoalHandleGripperCommand = rclcpp_action::ServerGoalHandle<control_msgs::action::GripperCommand>;

        //! Create an action server for GripperAction action and bind Goal callbacks.
        //! @param actionName Name of the action, similar to topic or service name.
        //! @param entityId entity which will execute callbacks through GripperRequestBus.
        GripperActionServer(const AZStd::string& actionName, const AZ::EntityId& entityId);

        //! Cancel the current goal.
        //! @param result Result to be passed to through action server to the client.
        void CancelGoal(std::shared_ptr<GripperCommand::Result> result);

        //! Report goal success to the action server.
        //! @param result Result which contains success code.
        void GoalSuccess(std::shared_ptr<GripperCommand::Result> result);

        //! Publish feedback during an active action.
        //! @param feedback An action feedback message informing about the progress.
        void PublishFeedback(std::shared_ptr<GripperCommand::Feedback> feedback);

        //! Check if the goal is in an active state
        bool IsGoalActiveState() const;

        //! Check if the goal is in a pending state
        bool IsReadyForExecution() const;

    private:
        rclcpp_action::GoalResponse GoalReceivedCallback(
            const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const GripperCommand::Goal> goal);
        rclcpp_action::CancelResponse GoalCancelledCallback(const std::shared_ptr<GoalHandleGripperCommand> goal_handle);
        void GoalAcceptedCallback(const std::shared_ptr<GoalHandleGripperCommand> goal_handle);

        rclcpp_action::Server<GripperCommand>::SharedPtr actionServer;
        std::shared_ptr<GoalHandleGripperCommand> m_goalHandle;
        AZ::EntityId m_entityId; //! Entity that has target gripper component
    };
} // namespace ROS2
