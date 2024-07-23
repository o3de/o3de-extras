/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <RobotImporter/SDFormat/ROS2SDFormatHooksUtils.h>
#include <RobotImporter/SDFormat/ROS2ModelPluginHooks.h>
#include <RobotImporter/Utils/RobotImporterUtils.h>
#include <AzCore/std/string/string.h>
#include <AzCore/std/containers/vector.h>
#include <Manipulation/Controllers/JointsArticulationControllerComponent.h>
#include <Manipulation/JointsManipulationEditorComponent.h>

namespace ROS2::SDFormat
{
    namespace StatePublisherUtils
    {
        // Find all parent links in model and return pointers to their entities
        AZ::Entity* GetParentLinkEntity(const sdf::Model& sdfModel, const CreatedEntitiesMap& createdEntities) {

            auto allLinks = Utils::GetAllLinks(sdfModel);

            for (auto &link : allLinks)
            {
                AZStd::string linkName = link.first;
                if (linkName.find_last_of(':') != std::string::npos)
                {
                    int nameBeginning = linkName.find_last_of(':') + 1;
                    linkName = linkName.substr(nameBeginning, linkName.size() - nameBeginning);
                }
                if (!Utils::IsChildLink(sdfModel, linkName))
                {
                    auto entityId = HooksUtils::GetLinkEntityId(std::string(linkName.c_str(), linkName.size()), sdfModel, createdEntities);
                    if (entityId.IsValid())
                    {
                        return AzToolsFramework::GetEntityById(entityId);
                    }
                }
            }

            return nullptr;
        } 
    } // namespace StatePublisherUtils

    ModelPluginImporterHook ROS2ModelPluginHooks::ROS2JointStatePublisherModel()
    {
        ModelPluginImporterHook importerHook;
        importerHook.m_pluginNames = AZStd::unordered_set<AZStd::string>{ "libgazebo_ros_joint_state_publisher.so" };
        importerHook.m_supportedPluginParams = AZStd::unordered_set<AZStd::string>{ ">update_rate", ">ros>namespace", ">ros>argument", ">ros>remapping" };

        importerHook.m_sdfPluginToComponentCallback =
            [](AZ::Entity& entity, const sdf::Plugin& sdfPlugin, const sdf::Model& sdfModel, const CreatedEntitiesMap& createdEntities)
            -> ModelPluginImporterHook::ConvertPluginOutcome
        {
            auto parentLinkEntity = StatePublisherUtils::GetParentLinkEntity(sdfModel, createdEntities);

            HooksUtils::PluginParams statePublisherParams = HooksUtils::GetPluginParams(sdfPlugin);

            // add components necessary for publishing to parent link
            if (!HooksUtils::CreateComponent<JointsArticulationControllerComponent>(*parentLinkEntity))
            {
                return AZ::Failure(AZStd::string("Failed to create ROS2 Joint State Publisher"));
            }
            auto manipulationElement = HooksUtils::CreateComponent<JointsManipulationEditorComponent>(*parentLinkEntity);
            if (manipulationElement)
            {
                auto manipulationComponent = dynamic_cast<JointsManipulationEditorComponent*>(manipulationElement);
                PublisherConfiguration publisherConfiguration;
                if (statePublisherParams.contains("update_rate"))
                {
                    std::string freqFromPlugin(statePublisherParams["update_rate"].begin(), statePublisherParams["update_rate"].size());
                    publisherConfiguration.m_frequency = std::stof(freqFromPlugin);
                }

                publisherConfiguration.m_topicConfiguration.m_type = "sensor_msgs::msg::JointState";
                AZStd::string messageTopic = "joint_states";
                if (statePublisherParams.contains("joint_states"))
                {
                    messageTopic = HooksUtils::PluginParser::LastOnPath(statePublisherParams["joint_states"]);
                }
                publisherConfiguration.m_topicConfiguration.m_topic = messageTopic;
                manipulationComponent->SetPublisherConfiguration(publisherConfiguration);
                return AZ::Success();
            }
            else
            {
                return AZ::Failure(AZStd::string("Failed to create ROS2 Joint State Publisher"));
            }
        };

        return importerHook;
    }
} // namespace ROS2::SDFormat