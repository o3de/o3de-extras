/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "JointsManipulationEditorComponent.h"
#include "JointUtils.h"
#include "JointsManipulationComponent.h"
#include <AzCore/Component/ComponentApplicationBus.h>
#include <AzCore/Component/TransformBus.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/std/functional.h>
#include <AzToolsFramework/API/ToolsApplicationAPI.h>
#include <ROS2/Frame/ROS2FrameComponentBus.h>
#include <ROS2/Frame/ROS2FrameEditorComponentBus.h>
#include <ROS2Controllers/Manipulation/Controllers/JointsPositionControllerRequests.h>
#include <ROS2Controllers/Manipulation/JointInfo.h>
#include <Source/ArticulationLinkComponent.h>
#include <Source/HingeJointComponent.h>

namespace ROS2Controllers
{
    JointsManipulationEditorComponent::JointsManipulationEditorComponent()
    {
        m_jointStatePublisherConfiguration.m_topicConfiguration.m_type = "sensor_msgs::msg::JointState";
        m_jointStatePublisherConfiguration.m_topicConfiguration.m_topic = "joint_states";
        m_jointStatePublisherConfiguration.m_frequency = 25.0f;
    }

    JointsManipulationEditorComponent::JointsManipulationEditorComponent(const ROS2::PublisherConfiguration& publisherConfiguration)
    {
        m_jointStatePublisherConfiguration = publisherConfiguration;
    }

    void JointsManipulationEditorComponent::BuildGameEntity(AZ::Entity* gameEntity)
    {
        gameEntity->CreateComponent<JointsManipulationComponent>(m_jointStatePublisherConfiguration, m_initialPositions);
    }

    void JointsManipulationEditorComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("ROS2Frame"));
        required.push_back(AZ_CRC_CE("JointsControllerService"));
    }

    void JointsManipulationEditorComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("JointsManipulationService"));
    }

    void JointsManipulationEditorComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("JointsManipulationService"));
    }

    void JointsManipulationEditorComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<JointsManipulationEditorComponent, AZ::Component>()
                ->Version(1)
                ->Field("JointStatePublisherConfiguration", &JointsManipulationEditorComponent::m_jointStatePublisherConfiguration)
                ->Field("Initial positions", &JointsManipulationEditorComponent::m_initialPositions);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<JointsManipulationEditorComponent>("JointsManipulationEditorComponent", "Component for manipulation of joints")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->Attribute(AZ::Edit::Attributes::Icon, "Editor/Icons/Components/JointsManipulationEditorComponent.svg")
                    ->Attribute(
                        AZ::Edit::Attributes::ViewportIcon, "Editor/Icons/Components/Viewport/JointsManipulationEditorComponent.svg")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &JointsManipulationEditorComponent::m_jointStatePublisherConfiguration,
                        "Joint State Publisher",
                        "Configuration of Joint State Publisher")
                    ->UIElement(AZ::Edit::UIHandlers::Button, "Reload joints", "Reload joints")
                    ->Attribute(AZ::Edit::Attributes::ButtonText, "Reload joints")
                    ->Attribute(AZ::Edit::Attributes::ChangeNotify, &JointsManipulationEditorComponent::ReloadJoints)
                    ->Attribute(AZ::Edit::Attributes::NameLabelOverride, "")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &JointsManipulationEditorComponent::m_initialPositions,
                        "Initial positions",
                        "Initial positions of all the joints (in order)");
            }
        }
    }

    AZ::Crc32 JointsManipulationEditorComponent::ReloadJoints()
    {
        AzToolsFramework::ScopedUndoBatch undo("ReloadJoints");
        // create backup of the configuration
        AZStd::unordered_map<AZStd::string, float> configBackup;
        for (const auto& [jointName, jointValue] : m_initialPositions)
        {
            configBackup[jointName] = jointValue;
        }
        m_initialPositions.clear();

        AZStd::set<AZ::EntityId> childrenEntityIds;
        ROS2::ROS2FrameEditorComponentBus::EventResult(
            childrenEntityIds, GetEntityId(), &ROS2::ROS2FrameEditorComponentBus::Events::GetFrameDescendants);

        for (const auto& entityId : childrenEntityIds)
        {
            AZ::Entity* childEntity = nullptr;
            AZ::ComponentApplicationBus::BroadcastResult(childEntity, &AZ::ComponentApplicationBus::Events::FindEntity, entityId);
            AZ_Assert(childEntity, "Entity not found!");

            AZ::Name jointName;
            ROS2::ROS2FrameComponentBus::EventResult(
                jointName, childEntity->GetId(), &ROS2::ROS2FrameComponentBus::Events::GetNamespacedJointName);

            const AZStd::string jointNameStr = jointName.GetCStr();
            const bool hasNonFixedJoints = JointUtils::HasNonFixedJoints(childEntity);
            if (!jointNameStr.empty() && hasNonFixedJoints)
            {
                m_initialPositions.emplace_back(AZStd::make_pair(jointNameStr, configBackup[jointNameStr]));
            }
        }
        undo.MarkEntityDirty(GetEntity()->GetId());
        return AZ::Edit::PropertyRefreshLevels::EntireTree;
    }
} // namespace ROS2Controllers
