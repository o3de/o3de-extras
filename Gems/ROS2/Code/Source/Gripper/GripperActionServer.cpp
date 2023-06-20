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
            AZStd::bind(&GripperActionServer::handleGoal, this, AZStd::placeholders::_1, AZStd::placeholders::_2),
            AZStd::bind(&GripperActionServer::handleCancel, this, AZStd::placeholders::_1),
            AZStd::bind(&GripperActionServer::handleAccepted, this, AZStd::placeholders::_1));
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

    void GripperActionServer::OnTick(float delta, AZ::ScriptTimePoint timePoint)
    {
        if (m_goalHandle && IsGoalActiveState())
        {
            auto feedback = std::make_shared<GripperCommand::Feedback>();
            float position = 0.0;
            float effort = 0.0;
            bool isDone = false;

            GripperRequestBus::EventResult(position, GetEntityId(), &GripperRequestBus::Events::GetGripperPosition);
            GripperRequestBus::EventResult(effort, GetEntityId(), &GripperRequestBus::Events::GetGripperEffort);
            GripperRequestBus::EventResult(isDone, GetEntityId(), &GripperRequestBus::Events::IsGripperReachedGoal);

            if (isDone)
            {
                AZ_Printf("GripperActionServer::OnTick", "GripperActionServer::OnTick: Gripper reached goal!");
                auto result = std::make_shared<GripperCommand::Result>();
                result->position = position;
                result->effort = effort;
                result->reached_goal = true;
                m_goalHandle->succeed(result);
            }
            else
            {
                feedback->position = position;
                feedback->position = effort;
                feedback->reached_goal = false;
                m_goalHandle->publish_feedback(feedback);
            }
        }
    }

    rclcpp_action::GoalResponse GripperActionServer::handleGoal(
        const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const GripperCommand::Goal> goal)
    {
        (void)uuid;
        AZ::Outcome<void, AZStd::string> commandOutcome = AZ::Failure(AZStd::string("No gripper component found!"));
        GripperRequestBus::EventResult(
            commandOutcome, GetEntityId(), &GripperRequestBus::Events::GripperCommand, goal->command.position, goal->command.max_effort);
        if (!commandOutcome.IsSuccess())
        {
            AZ_TracePrintf(
                "GripperActionServer::handleGoal",
                "GripperActionServer::handleGoal: GripperCommand could not be accepted: %s",
                commandOutcome.GetError().c_str());
            return rclcpp_action::GoalResponse::REJECT;
        }
        AZ_TracePrintf("GripperActionServer::handleGoal", "GripperActionServer::handleGoal: GripperCommand accepted!");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse GripperActionServer::handleCancel(const std::shared_ptr<GoalHandleGripperCommand> goal_handle)
    {
        AZ_Assert(m_goalHandle, "Invalid goal handle!");
        if (m_goalHandle)
        {
            AZ_TracePrintf("GripperActionServer::handleCancel", "Trying to cancel goal");
            AZ::Outcome<void, AZStd::string> cancelOutcome = AZ::Failure(AZStd::string("No gripper component found!"));
            GripperRequestBus::EventResult(cancelOutcome, GetEntityId(), &GripperRequestBus::Events::CancelGripperCommand);
            {
                AZ_TracePrintf(
                    "GripperActionServer::handleCancel", "Cancelling could not be accepted: %s", cancelOutcome.GetError().c_str());
                return rclcpp_action::CancelResponse::REJECT;
            }
            return rclcpp_action::CancelResponse::ACCEPT;
            m_goalHandle = nullptr;
        }
        m_goalHandle = nullptr;
        return rclcpp_action::CancelResponse::REJECT;
    }

    void GripperActionServer::handleAccepted(const std::shared_ptr<GoalHandleGripperCommand> goal_handle)
    {
        m_goalHandle = goal_handle;
    }

} // namespace ROS2
