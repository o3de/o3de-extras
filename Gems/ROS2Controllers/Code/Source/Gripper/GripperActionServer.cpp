/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root
 * of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "Utils.h"

#include "GripperActionServer.h"
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzFramework/Components/TransformComponent.h>
#include <ROS2/ROS2Bus.h>
#include <ROS2/ROS2GemUtilities.h>

namespace ROS2
{
    GripperActionServer::GripperActionServer(const AZStd::string& actionName, const AZ::EntityId& entityId)
        : m_entityId(entityId)
    {
        auto ros2Node = ROS2Interface::Get()->GetNode();
        actionServer = rclcpp_action::create_server<GripperCommand>(
            ros2Node,
            actionName.data(),
            AZStd::bind(&GripperActionServer::GoalReceivedCallback, this, AZStd::placeholders::_1, AZStd::placeholders::_2),
            AZStd::bind(&GripperActionServer::GoalCancelledCallback, this, AZStd::placeholders::_1),
            AZStd::bind(&GripperActionServer::GoalAcceptedCallback, this, AZStd::placeholders::_1));
    }

    bool GripperActionServer::IsGoalActiveState() const
    {
        if (!m_goalHandle)
        {
            return false;
        }
        return m_goalHandle->is_active() || m_goalHandle->is_executing() || m_goalHandle->is_canceling();
    }

    bool GripperActionServer::IsReadyForExecution() const
    {
        // Has no goal handle yet - can be accepted.
        if (!m_goalHandle)
        {
            return true;
        }
        // Accept if the previous one is in a terminal state.
        return IsGoalActiveState() == false;
    }

    rclcpp_action::GoalResponse GripperActionServer::GoalReceivedCallback(
        [[maybe_unused]] const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const GripperCommand::Goal> goal)
    {
        if (!IsReadyForExecution())
        {
            AZ_Printf("GripperActionServer", "GripperActionServer::handleGoal: Rejected goal, already executing\n");
            if (m_goalHandle)
            {
                AZ_Trace(
                    "GripperActionServer",
                    " is_active: %d,  is_executing %d, is_canceling : %d\n",
                    m_goalHandle->is_active(),
                    m_goalHandle->is_executing(),
                    m_goalHandle->is_canceling());
            }
            return rclcpp_action::GoalResponse::REJECT;
        }

        AZ::Outcome<void, AZStd::string> commandOutcome = AZ::Failure(AZStd::string("No gripper component found!"));
        GripperRequestBus::EventResult(
            commandOutcome, m_entityId, &GripperRequestBus::Events::GripperCommand, goal->command.position, goal->command.max_effort);
        if (!commandOutcome.IsSuccess())
        {
            AZ_Trace("GripperActionServer", "GripperCommand could not be accepted: %s\n", commandOutcome.GetError().c_str());
            return rclcpp_action::GoalResponse::REJECT;
        }
        AZ_Trace("GripperActionServer", "GripperActionServer::handleGoal: GripperCommand accepted!\n");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse GripperActionServer::GoalCancelledCallback(const std::shared_ptr<GoalHandleGripperCommand> goal_handle)
    {
        AZ::Outcome<void, AZStd::string> cancelOutcome = AZ::Failure(AZStd::string("No gripper component found!"));
        GripperRequestBus::EventResult(cancelOutcome, m_entityId, &GripperRequestBus::Events::CancelGripperCommand);

        if (!cancelOutcome)
        { // This will not happen in a simulation unless intentionally done for behavior validation
            AZ_Trace("GripperActionServer", "Cancelling could not be accepted: %s\n", cancelOutcome.GetError().c_str());
            return rclcpp_action::CancelResponse::REJECT;
        }
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void GripperActionServer::GoalAcceptedCallback(const std::shared_ptr<GoalHandleGripperCommand> goal_handle)
    {
        AZ_Trace("GripperActionServer", "Goal accepted!\n");
        m_goalHandle = goal_handle;
    }

    void GripperActionServer::CancelGoal(std::shared_ptr<GripperCommand::Result> result)
    {
        AZ_Assert(m_goalHandle, "Invalid goal handle!");
        if (m_goalHandle && m_goalHandle->is_canceling())
        {
            AZ_Trace("GripperActionServer", "Cancelling goal\n");
            m_goalHandle->canceled(result);
        }
    }

    void GripperActionServer::GoalSuccess(std::shared_ptr<GripperCommand::Result> result)
    {
        AZ_Assert(m_goalHandle, "Invalid goal handle!");
        if (m_goalHandle && (m_goalHandle->is_executing() || m_goalHandle->is_canceling()))
        {
            AZ_Trace("GripperActionServer", "Goal succeeded\n");
            m_goalHandle->succeed(result);
        }
    }

    void GripperActionServer::PublishFeedback(std::shared_ptr<GripperCommand::Feedback> feedback)
    {
        AZ_Assert(m_goalHandle, "Invalid goal handle!");
        if (m_goalHandle && m_goalHandle->is_executing())
        {
            m_goalHandle->publish_feedback(feedback);
        }
    }

} // namespace ROS2
