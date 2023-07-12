/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root
 * of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "Utils.h"

#include "GripperActionServerComponent.h"
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzFramework/Components/TransformComponent.h>

namespace ROS2
{
    void GripperActionServerComponent::Activate()
    {
        auto* ros2Frame = Utils::GetGameOrEditorComponent<ROS2FrameComponent>(GetEntity());
        AZ_Assert(ros2Frame, "Missing Frame Component!");
        AZStd::string namespacedAction = ROS2Names::GetNamespacedName(ros2Frame->GetNamespace(), m_gripperActionServerName);
        AZ_Printf("GripperActionServerComponent", "Creating Gripper Action Server: %s\n", namespacedAction.c_str());
        m_gripperActionServer = AZStd::make_unique<GripperActionServer>(namespacedAction, GetEntityId());
        AZ::TickBus::Handler::BusConnect();
    }

    void GripperActionServerComponent::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
        m_gripperActionServer.reset();
    }

    void GripperActionServerComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<GripperActionServerComponent, AZ::Component>()
                ->Field("ActionServerName", &GripperActionServerComponent::m_gripperActionServerName)
                ->Version(1);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<GripperActionServerComponent>("GripperActionServer", "Component for the gripper action server")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "GripperActionServer")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &GripperActionServerComponent::m_gripperActionServerName,
                        "Gripper Action Server",
                        "Action name for the gripper server.");
            }
        }
    }

    void GripperActionServerComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("ROS2Frame"));
    }

    std::shared_ptr<GripperActionServer::GripperCommand::Feedback> GripperActionServerComponent::ProduceFeedback() const
    {
        auto feedback = std::make_shared<GripperCommand::Feedback>();
        float position = 0.0f;
        float effort = 0.0f;
        bool stalled = false;
        GripperRequestBus::EventResult(position, GetEntityId(), &GripperRequestBus::Events::GetGripperPosition);
        GripperRequestBus::EventResult(effort, GetEntityId(), &GripperRequestBus::Events::GetGripperEffort);
        GripperRequestBus::EventResult(stalled, GetEntityId(), &GripperRequestBus::Events::IsGripperNotMoving);
        feedback->position = position;
        feedback->position = effort;
        feedback->reached_goal = false;
        feedback->stalled = stalled;
        return feedback;
    }

    std::shared_ptr<GripperActionServer::GripperCommand::Result> GripperActionServerComponent::ProduceResult() const
    {
        auto result = std::make_shared<GripperCommand::Result>();
        float position = 0.0f;
        float effort = 0.0f;
        bool stalled = false;
        GripperRequestBus::EventResult(position, GetEntityId(), &GripperRequestBus::Events::GetGripperPosition);
        GripperRequestBus::EventResult(effort, GetEntityId(), &GripperRequestBus::Events::GetGripperEffort);
        GripperRequestBus::EventResult(stalled, GetEntityId(), &GripperRequestBus::Events::IsGripperNotMoving);
        result->position = position;
        result->position = effort;
        result->reached_goal = true;
        result->stalled = stalled;
        return result;
    }

    void GripperActionServerComponent::OnTick(float delta, AZ::ScriptTimePoint timePoint)
    {
        AZ_Assert(m_gripperActionServer, "GripperActionServer::OnTick: GripperActionServer is null!");
        if (!m_gripperActionServer->IsGoalActiveState())
        {
            return;
        }
        bool isDone = false;
        bool isCancelled = false;
        GripperRequestBus::EventResult(isDone, GetEntityId(), &GripperRequestBus::Events::HasGripperReachedGoal);
        GripperRequestBus::EventResult(isCancelled, GetEntityId(), &GripperRequestBus::Events::HasGripperCommandBeenCancelled);
        if (isCancelled)
        {
            m_gripperActionServer->CancelGoal(ProduceResult());
            return;
        }
        if (isDone)
        {
            AZ_Printf("GripperActionServer::OnTick", "GripperActionServer::OnTick: Gripper reached goal!");
            m_gripperActionServer->GoalSuccess(ProduceResult());
            return;
        }
        m_gripperActionServer->PublishFeedback(ProduceFeedback());
        return;
    }

} // namespace ROS2
