
// {BEGIN_LICENSE}
/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
// {END_LICENSE}

#include "${Name}SampleComponent.h"
#include <AzCore/Component/ComponentApplicationBus.h>
#include <AzCore/Component/TransformBus.h>
#include <AzCore/Math/Transform.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>

#include <imgui/imgui.h>

#include <ROS2/ROS2Bus.h>
#include <ROS2/ROS2GemUtilities.h>
#include <ROS2/Utilities/ROS2Conversions.h>
#include <ROS2/Utilities/ROS2Names.h>

namespace ${SanitizedCppName}
{

    ${SanitizedCppName}SampleComponent::${SanitizedCppName}SampleComponent()
    {
        m_goalTopicConfiguration.m_topic = "/goal_pose";
        m_goalTopicConfiguration.m_type = "geometry_msgs::msg::PoseStamped";
    }

    void ${SanitizedCppName}SampleComponent::Activate()
    {
        auto ros2Node = ROS2::ROS2Interface::Get()->GetNode();
        m_goalPublisher = ros2Node->create_publisher<geometry_msgs::msg::PoseStamped>(m_goalTopicConfiguration.m_topic.data(), m_goalTopicConfiguration.GetQoS());
        ImGui::ImGuiUpdateListenerBus::Handler::BusConnect();

    }

    void ${SanitizedCppName}SampleComponent::Deactivate()
    {
        m_goalPublisher.reset();
        ImGui::ImGuiUpdateListenerBus::Handler::BusDisconnect();
    }

    void ${SanitizedCppName}SampleComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<${SanitizedCppName}SampleComponent, AZ::Component>()
                ->Version(1)
                ->Field("goals", &${SanitizedCppName}SampleComponent::m_goalEntities)
                ->Field("goalMessageTopic", &${SanitizedCppName}SampleComponent::m_goalTopicConfiguration);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<${SanitizedCppName}SampleComponent>("${SanitizedCppName}SampleComponent", "A sample component that sends goal")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "${SanitizedCppName}SampleComponent")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "Sample ROS2 Project")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &${SanitizedCppName}SampleComponent::m_goalEntities, "Goals", "Entities goals")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &${SanitizedCppName}SampleComponent::m_goalTopicConfiguration,
                        "Topic for goal message",
                        "Configuration for ROS2 topic to send goal message to");
            }
        }
    }

    void ${SanitizedCppName}SampleComponent::OnImGuiUpdate()
    {
        ImGui::Begin("${SanitizedCppName}SampleComponent");
        for (const auto& entityId : m_goalEntities)
        {
            AZStd::string entityName;
            AZ::ComponentApplicationBus::BroadcastResult(entityName, &AZ::ComponentApplicationRequests::GetEntityName, entityId);
            const AZStd::string buttonName = AZStd::string::format("Send goal %s", entityName.c_str());

            if (ImGui::Button(buttonName.c_str()))
            {
                AZ::Transform transform;
                AZ::TransformBus::EventResult(transform, entityId, &AZ::TransformBus::Events::GetWorldTM);
                geometry_msgs::msg::PoseStamped poseStamped;
                poseStamped.header.frame_id = "odom";
                poseStamped.pose = ROS2::ROS2Conversions::ToROS2Pose(transform);
                m_goalPublisher->publish(poseStamped);
            }
        }
        ImGui::End();
    }
} // namespace ${SanitizedCppName}