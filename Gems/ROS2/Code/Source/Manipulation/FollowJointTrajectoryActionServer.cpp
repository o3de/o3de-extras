/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "FollowJointTrajectoryActionServer.h"
#include <AzCore/std/functional.h>
#include <ROS2/ROS2Bus.h>

namespace ROS2
{
    FollowJointTrajectoryActionServer::FollowJointTrajectoryActionServer(const AZStd::string& actionName, const AZ::EntityId& entityId)
        : m_entityId(entityId)
    {
        m_actionServer = rclcpp_action::create_server<FollowJointTrajectory>(
            ROS2Interface::Get()->GetNode(),
            actionName.c_str(),
            AZStd::bind(&FollowJointTrajectoryActionServer::GoalReceivedCallback, this, AZStd::placeholders::_1, AZStd::placeholders::_2),
            AZStd::bind(&FollowJointTrajectoryActionServer::GoalCancelledCallback, this, AZStd::placeholders::_1),
            AZStd::bind(&FollowJointTrajectoryActionServer::GoalAcceptedCallback, this, AZStd::placeholders::_1));
    }

    JointsTrajectoryRequests::TrajectoryActionStatus FollowJointTrajectoryActionServer::GetGoalStatus() const
    {
        return m_goalStatus;
    }

    void FollowJointTrajectoryActionServer::SetGoalSuccess()
    {
        m_goalStatus = JointsTrajectoryRequests::TrajectoryActionStatus::Succeeded;
    }

    void FollowJointTrajectoryActionServer::CancelGoal(std::shared_ptr<FollowJointTrajectory::Result> result)
    {
        AZ_Assert(m_goalHandle, "Invalid goal handle!");
        if (m_goalHandle && m_goalHandle->is_canceling())
        {
            AZ_Trace("FollowJointTrajectoryActionServer", "Cancelling goal\n");
            m_goalHandle->canceled(result);
        }
    }

    void FollowJointTrajectoryActionServer::GoalSuccess(std::shared_ptr<FollowJointTrajectory::Result> result)
    {
        AZ_Assert(m_goalHandle, "Invalid goal handle!");
        if (m_goalHandle && m_goalHandle->is_executing())
        {
            AZ_Trace("FollowJointTrajectoryActionServer", "Goal succeeded\n");
            m_goalHandle->succeed(result);
            m_goalStatus = JointsTrajectoryRequests::TrajectoryActionStatus::Succeeded;
        }
    }

    void FollowJointTrajectoryActionServer::PublishFeedback(std::shared_ptr<FollowJointTrajectory::Feedback> feedback)
    {
        AZ_Assert(m_goalHandle, "Invalid goal handle!");
        if (m_goalHandle && m_goalHandle->is_executing())
        {
            m_goalHandle->publish_feedback(feedback);
        }
    }

    bool FollowJointTrajectoryActionServer::IsGoalActiveState() const
    {
        return m_goalHandle->is_active() || m_goalHandle->is_executing() || m_goalHandle->is_canceling();
    }

    bool FollowJointTrajectoryActionServer::IsReadyForExecution() const
    {
        // Has a goal handle yet - can be accepted.
        if (!m_goalHandle)
        {
            return true;
        }
        // accept goal if previous is terminal state
        return IsGoalActiveState() == false;
    }

    bool FollowJointTrajectoryActionServer::IsExecuting() const
    {
        return m_goalHandle && m_goalHandle->is_executing();
    }

    rclcpp_action::GoalResponse FollowJointTrajectoryActionServer::GoalReceivedCallback(
        [[maybe_unused]] const rclcpp_action::GoalUUID& uuid, [[maybe_unused]] std::shared_ptr<const FollowJointTrajectory::Goal> goal)
    { // Accept each received goal. It will be aborted if other goal is active (no deferring/queuing).
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse FollowJointTrajectoryActionServer::GoalCancelledCallback(
        [[maybe_unused]] const std::shared_ptr<GoalHandle> goalHandle)
    { // Accept each cancel attempt
        AZ::Outcome<void, AZStd::string> cancelOutcome;
        JointsTrajectoryRequestBus::EventResult(cancelOutcome, m_entityId, &JointsTrajectoryRequests::CancelTrajectoryGoal);

        if (!cancelOutcome)
        { // This will not happen in simulation unless intentionally done for behavior validation
            AZ_Trace("FollowJointTrajectoryActionServer", "Cancelling could not be accepted: %s\n", cancelOutcome.GetError().c_str());
            return rclcpp_action::CancelResponse::REJECT;
        }

        m_goalStatus = JointsTrajectoryRequests::TrajectoryActionStatus::Cancelled;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void FollowJointTrajectoryActionServer::GoalAcceptedCallback(const std::shared_ptr<GoalHandle> goalHandle)
    {
        AZ_Trace("FollowJointTrajectoryActionServer", "Goal accepted\n");

        if (!IsReadyForExecution())
        {
            AZ_Trace("FollowJointTrajectoryActionServer", "Goal aborted: server is not ready for execution!");
            if (m_goalHandle)
            {
                AZ_Trace(
                    "FollowJointTrajectoryActionServer",
                    " is_active: %d,  is_executing %d, is_canceling : %d",
                    m_goalHandle->is_active(),
                    m_goalHandle->is_executing(),
                    m_goalHandle->is_canceling());
            }

            auto result = std::make_shared<FollowJointTrajectory::Result>();
            result->error_string = "Goal aborted: server is not ready for execution!";
            result->error_code = FollowJointTrajectory::Result::INVALID_GOAL;
            goalHandle->abort(result);
            return;
        }

        AZ::Outcome<void, FollowJointTrajectory::Result> executionOrderOutcome;
        JointsTrajectoryRequestBus::EventResult(
            executionOrderOutcome, m_entityId, &JointsTrajectoryRequests::StartTrajectoryGoal, goalHandle->get_goal());

        if (!executionOrderOutcome)
        {
            AZ_Trace(
                "FollowJointTrajectoryActionServer",
                "Execution was not accepted: %s",
                executionOrderOutcome.GetError().error_string.c_str());

            auto result = std::make_shared<FollowJointTrajectory::Result>(executionOrderOutcome.GetError());

            goalHandle->abort(result);
            return;
        }

        m_goalHandle = goalHandle;
        // m_goalHandle->execute(); // No need to call this, as we are already executing the goal due to ACCEPT_AND_EXECUTE
        m_goalStatus = JointsTrajectoryRequests::TrajectoryActionStatus::Executing;
    }
} // namespace ROS2
