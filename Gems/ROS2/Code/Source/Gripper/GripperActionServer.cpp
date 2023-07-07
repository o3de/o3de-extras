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

namespace ROS2
{
    void GripperActionServer::Activate()
    {
        auto ros2Node = ROS2Interface::Get()->GetNode();

        actionServer = rclcpp_action::create_server<GripperCommand>(
            ros2Node,
            m_gripperActionServerName.data(),
            AZStd::bind(&GripperActionServer::GoalReceivedCallback, this, AZStd::placeholders::_1, AZStd::placeholders::_2),
            AZStd::bind(&GripperActionServer::GoalCancelledCallback, this, AZStd::placeholders::_1),
            AZStd::bind(&GripperActionServer::GoalAcceptedCallback, this, AZStd::placeholders::_1));
        AZ::TickBus::Handler::BusConnect();
    }

    void GripperActionServer::Deactivate()
    {
        actionServer.reset();
        AZ::TickBus::Handler::BusDisconnect();
    }

    void GripperActionServer::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<GripperActionServer, AZ::Component>()
                ->Field("ActionServerName", &GripperActionServer::m_gripperActionServerName)
                ->Version(1);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<GripperActionServer>("GripperActionServer", "GripperActionServer")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "GripperActionServer")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->DataElement(
                        AZ::Edit::UIHandlers::EntityId,
                        &GripperActionServer::m_gripperActionServerName,
                        "Gripper Action Server",
                        "Gripper Action Server.");
            }
        }
    }

    bool GripperActionServer::IsGoalActiveState() const
    {
        return m_goalHandle->is_active() || m_goalHandle->is_executing() || m_goalHandle->is_canceling();
    }

    bool GripperActionServer::IsReadyForExecution() const
    {
        // Has a goal handle yet - can be accepted.
        if (!m_goalHandle)
        {
            return true;
        }
        // accept goal if previous is terminal state
        return IsGoalActiveState() == false;
    }

    std::shared_ptr<GripperActionServer::GripperCommand::Feedback> GripperActionServer::ProduceFeedback() const
    {
        auto feedback = std::make_shared<GripperCommand::Feedback>();
        float position = 0.0;
        float effort = 0.0;
        GripperRequestBus::EventResult(position, GetEntityId(), &GripperRequestBus::Events::GetGripperPosition);
        GripperRequestBus::EventResult(effort, GetEntityId(), &GripperRequestBus::Events::GetGripperEffort);
        feedback->position = position;
        feedback->position = effort;
        feedback->reached_goal = false;
        return feedback;
    }

    std::shared_ptr<GripperActionServer::GripperCommand::Result> GripperActionServer::ProduceResult() const
    {
        auto result = std::make_shared<GripperCommand::Result >();
        float position = 0.0;
        float effort = 0.0;
        GripperRequestBus::EventResult(position, GetEntityId(), &GripperRequestBus::Events::GetGripperPosition);
        GripperRequestBus::EventResult(effort, GetEntityId(), &GripperRequestBus::Events::GetGripperEffort);
        result->position = position;
        result->position = effort;
        result->reached_goal = true;
        return result;
    }


    void GripperActionServer::OnTick(float delta, AZ::ScriptTimePoint timePoint)
    {
        if (m_goalHandle && IsGoalActiveState())
        {
            bool isDone = false;
            GripperRequestBus::EventResult(isDone, GetEntityId(), &GripperRequestBus::Events::HasGripperReachedGoal);

            if (isDone)
            {
                AZ_Printf("GripperActionServer::OnTick", "GripperActionServer::OnTick: Gripper reached goal!");
                auto result = ProduceResult();
                m_goalHandle->succeed(result);
            }
            else if (m_goalHandle->is_canceling())
            {
                m_goalHandle->canceled(ProduceResult());
            }
            else
            {
                auto feedback = ProduceFeedback();
                m_goalHandle->publish_feedback(feedback);
            }
        }
    }

    rclcpp_action::GoalResponse GripperActionServer::GoalReceivedCallback(
        [[maybe_unused]] const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const GripperCommand::Goal> goal)
    {
        if (!IsReadyForExecution())
        {
            AZ_Trace("GripperActionServer", "GripperActionServer::handleGoal: Rejected goal, already executing\n");
            if (m_goalHandle)
            {
                AZ_Trace(
                    "GripperActionServer",
                    " is_active: %d,  is_executing %d, is_canceling : %d",
                    m_goalHandle->is_active(),
                    m_goalHandle->is_executing(),
                    m_goalHandle->is_canceling());
            }
            return rclcpp_action::GoalResponse::REJECT;
        }

        AZ::Outcome<void, AZStd::string> commandOutcome = AZ::Failure(AZStd::string("No gripper component found!"));
        GripperRequestBus::EventResult(
            commandOutcome, GetEntityId(), &GripperRequestBus::Events::GripperCommand, goal->command.position, goal->command.max_effort);
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
        GripperRequestBus::EventResult(cancelOutcome, GetEntityId(), &GripperRequestBus::Events::CancelGripperCommand);

        if (!cancelOutcome)
        { // This will not happen in simulation unless intentionally done for behavior validation
            AZ_Trace("GripperActionServer", "Cancelling could not be accepted: %s\n", cancelOutcome.GetError().c_str());
            return rclcpp_action::CancelResponse::REJECT;
        }
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void GripperActionServer::GoalAcceptedCallback(const std::shared_ptr<GoalHandleGripperCommand> goal_handle)
    {
        AZ_Trace("GripperActionServer", "Goal Accepted accepted!\n");
        m_goalHandle = goal_handle;
    }

} // namespace ROS2
