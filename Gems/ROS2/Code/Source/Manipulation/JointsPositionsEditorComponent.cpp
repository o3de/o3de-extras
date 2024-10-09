/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root
 * of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "JointsPositionsEditorComponent.h"
#include "JointPositionsSubscriptionHandler.h"
#include "JointsPositionsComponent.h"

#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>

#include <AzToolsFramework/ToolsComponents/EditorComponentBase.h>
#include <ROS2/Frame/ROS2FrameEditorComponent.h>
#include <ROS2/Manipulation/JointsManipulationRequests.h>
#include <ROS2/ROS2GemUtilities.h>

namespace ROS2
{
    JointsPositionsEditorComponent::JointsPositionsEditorComponent()
    {
        m_topicConfiguration.m_type = "std_msgs::msg::Float64MultiArray";
        m_topicConfiguration.m_topic = "/position_controller/commands";
    }

    void JointsPositionsEditorComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("ROS2Frame"));
        required.push_back(AZ_CRC_CE("JointsControllerService"));
    }

    void JointsPositionsEditorComponent::BuildGameEntity(AZ::Entity* gameEntity)
    {
        gameEntity->CreateComponent<JointsPositionsComponent>(m_topicConfiguration, m_jointNames);
    }

    void JointsPositionsEditorComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<JointsPositionsEditorComponent, AZ::Component>()
                ->Version(0)
                ->Field("topicConfiguration", &JointsPositionsEditorComponent::m_topicConfiguration)
                ->Field("jointNames", &JointsPositionsEditorComponent::m_jointNames);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<JointsPositionsEditorComponent>("JointsPositionsEditorComponent", "Component controlling a finger gripper.")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "JointsPositionsEditorComponent")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->Attribute(AZ::Edit::Attributes::Icon, "Editor/Icons/Components/JointsManipulationEditorComponent.svg")
                    ->Attribute(
                        AZ::Edit::Attributes::ViewportIcon, "Editor/Icons/Components/Viewport/JointsManipulationEditorComponent.svg")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &JointsPositionsEditorComponent::m_topicConfiguration,
                        "Topic configuration",
                        "Topic configuration of Joint Positions Component")
                    ->UIElement(AZ::Edit::UIHandlers::Button, "Find all joints", "Find all joints")
                    ->Attribute(AZ::Edit::Attributes::ButtonText, "Find all joints")
                    ->Attribute(AZ::Edit::Attributes::ChangeNotify, &JointsPositionsEditorComponent::FindAllJoints)
                    ->Attribute(AZ::Edit::Attributes::NameLabelOverride, "")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &JointsPositionsEditorComponent::m_jointNames,
                        "Joint names",
                        "Names of all the joints (in order)");
            }
        }
    }

    AZ::Crc32 JointsPositionsEditorComponent::FindAllJoints()
    {
        AZStd::function<void(const AZ::Entity* entity)> getAllJointsHierarchy = [&](const AZ::Entity* entity)
        {
            auto* frameEditorComponent =
                azrtti_cast<ROS2::ROS2FrameEditorComponent*>(Utils::GetGameOrEditorComponent<ROS2::ROS2FrameEditorComponent>(entity));
            AZ_Assert(frameEditorComponent, "ROS2FrameEditorComponent does not exist!");

            AZStd::string jointName(frameEditorComponent->GetJointName().GetCStr());
            if (!jointName.empty())
            {
                m_jointNames.emplace_back(jointName);
            }

            const auto& childrenEntityIds = frameEditorComponent->GetFrameChildren();
            if (!childrenEntityIds.empty())
            {
                for (const auto& entityId : childrenEntityIds)
                {
                    AZ::Entity* entity = nullptr;
                    AZ::ComponentApplicationBus::BroadcastResult(entity, &AZ::ComponentApplicationBus::Events::FindEntity, entityId);
                    getAllJointsHierarchy(entity);
                }
            }
        };
        getAllJointsHierarchy(GetEntity());

        return AZ::Edit::PropertyRefreshLevels::EntireTree;
    }

} // namespace ROS2
