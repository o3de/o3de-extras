/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "JointsManipulationEditorComponent.h"
#include "JointsManipulationComponent.h"
#include <AzCore/Component/ComponentApplicationBus.h>
#include <AzCore/Component/TransformBus.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/std/functional.h>
#include <ROS2/Frame/ROS2FrameEditorComponent.h>
#include <ROS2/Manipulation/Controllers/JointsPositionControllerRequests.h>
#include <ROS2/Manipulation/JointInfo.h>
#include <ROS2/ROS2GemUtilities.h>
#include <ROS2/Utilities/ROS2Names.h>
#include <Source/ArticulationLinkComponent.h>
#include <Source/HingeJointComponent.h>

namespace ROS2
{
    JointsManipulationEditorComponent::JointsManipulationEditorComponent()
    {
        m_jointStatePublisherConfiguration.m_topicConfiguration.m_type = "sensor_msgs::msg::JointState";
        m_jointStatePublisherConfiguration.m_topicConfiguration.m_topic = "joint_states";
        m_jointStatePublisherConfiguration.m_frequency = 25.0f;
    }

    void JointsManipulationEditorComponent::BuildGameEntity(AZ::Entity* gameEntity)
    {
        gameEntity->CreateComponent<JointsManipulationComponent>(
            m_jointStatePublisherConfiguration, m_initialPositions);
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
        // create backup of the configuration
        AZStd::unordered_map<AZStd::string, float> configBackup;
        for (const auto& [jointName, jointValue] : m_initialPositions)
        {
            configBackup[jointName] = jointValue;
        }
        m_initialPositions.clear();

        AZStd::function<void(const AZ::Entity* entity)> getAllJointsHierarchy = [&](const AZ::Entity* entity)
        {
            auto* frameEditorComponent =
                azrtti_cast<ROS2::ROS2FrameEditorComponent*>(Utils::GetGameOrEditorComponent<ROS2::ROS2FrameEditorComponent>(entity));
            AZ_Assert(frameEditorComponent, "ROS2FrameEditorComponent does not exist!");

            AZStd::string jointName(frameEditorComponent->GetJointName().GetCStr());
            if (!jointName.empty())
            {
                m_initialPositions.emplace_back(AZStd::make_pair(jointName, configBackup[jointName]));
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
