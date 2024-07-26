/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <AzCore/std/containers/vector.h>
#include <AzCore/std/string/string.h>
#include <Manipulation/Controllers/JointsArticulationControllerComponent.h>
#include <Manipulation/JointsManipulationEditorComponent.h>
#include <ROS2/Frame/ROS2FrameEditorComponent.h>
#include <RobotImporter/SDFormat/ROS2ModelPluginHooks.h>
#include <RobotImporter/SDFormat/ROS2SDFormatHooksUtils.h>
#include <RobotImporter/Utils/RobotImporterUtils.h>

namespace ROS2::SDFormat
{
    ModelPluginImporterHook ROS2ModelPluginHooks::ROS2JointStatePublisherModel()
    {
        ModelPluginImporterHook importerHook;
        importerHook.m_pluginNames = AZStd::unordered_set<AZStd::string>{ "libgazebo_ros_joint_state_publisher.so" };
        importerHook.m_supportedPluginParams = AZStd::unordered_set<AZStd::string>{ ">update_rate", ">ros>argument", ">ros>remapping" };

        importerHook.m_sdfPluginToComponentCallback =
            [](AZ::Entity& entity, const sdf::Plugin& sdfPlugin, const sdf::Model& sdfModel, const CreatedEntitiesMap& createdEntities)
            -> ModelPluginImporterHook::ConvertPluginOutcome
        {
            HooksUtils::PluginParams statePublisherParams = HooksUtils::GetPluginParams(sdfPlugin);

            // add required components
            HooksUtils::CreateComponent<JointsArticulationControllerComponent>(entity);
            HooksUtils::CreateComponent<ROS2FrameEditorComponent>(entity);

            auto manipulationElement = HooksUtils::CreateComponent<JointsManipulationEditorComponent>(entity);
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