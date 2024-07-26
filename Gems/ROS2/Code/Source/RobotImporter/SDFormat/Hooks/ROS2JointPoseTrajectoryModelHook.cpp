/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <Manipulation/Controllers/JointsArticulationControllerComponent.h>
#include <Manipulation/JointsManipulationEditorComponent.h>
#include <ROS2/Frame/ROS2FrameEditorComponent.h>
#include <Manipulation/JointsTrajectoryComponent.h>
#include <RobotImporter/SDFormat/ROS2ModelPluginHooks.h>
#include <RobotImporter/SDFormat/ROS2SDFormatHooksUtils.h>

namespace ROS2::SDFormat
{
    ModelPluginImporterHook ROS2ModelPluginHooks::ROS2JointPoseTrajectoryModel()
    {
        ModelPluginImporterHook importerHook;
        importerHook.m_pluginNames = AZStd::unordered_set<AZStd::string>{ "libgazebo_ros_joint_pose_trajectory.so" };
        importerHook.m_supportedPluginParams = AZStd::unordered_set<AZStd::string>{ ">topicName", ">ros>remapping", ">ros>argument", ">updateRate", ">update_rate" };

        importerHook.m_sdfPluginToComponentCallback =
            [](AZ::Entity& entity, const sdf::Plugin& sdfPlugin, const sdf::Model& sdfModel, const CreatedEntitiesMap& createdEntities)
            -> ModelPluginImporterHook::ConvertPluginOutcome
        {
            HooksUtils::PluginParams poseTrajectoryParams = HooksUtils::GetPluginParams(sdfPlugin);

            // add required components
            HooksUtils::CreateComponent<JointsArticulationControllerComponent>(entity);
            HooksUtils::CreateComponent<ROS2FrameEditorComponent>(entity);
            auto manipulationElement = HooksUtils::CreateComponent<JointsManipulationEditorComponent>(entity);
            if (manipulationElement)
            {
                auto manipulationComponent = dynamic_cast<JointsManipulationEditorComponent*>(manipulationElement);
                PublisherConfiguration publisherConfiguration;
                if (poseTrajectoryParams.contains("update_rate"))
                {
                    std::string freqFromPlugin(poseTrajectoryParams["update_rate"].begin(), poseTrajectoryParams["update_rate"].size());
                    publisherConfiguration.m_frequency = std::stof(freqFromPlugin);
                }
                else if (poseTrajectoryParams.contains("updateRate"))
                {
                    std::string freqFromPlugin(poseTrajectoryParams["updateRate"].begin(), poseTrajectoryParams["updateRate"].size());
                    publisherConfiguration.m_frequency = std::stof(freqFromPlugin);
                }
                publisherConfiguration.m_topicConfiguration.m_type = "sensor_msgs::msg::JointState";
                publisherConfiguration.m_topicConfiguration.m_topic = "joint_states";
                manipulationComponent->SetPublisherConfiguration(publisherConfiguration);
            }

            AZStd::string trajectoryActionName("arm_controller/follow_joint_trajectory");
            if (poseTrajectoryParams.contains("set_joint_trajectory"))
            {
                trajectoryActionName = HooksUtils::PluginParser::LastOnPath(poseTrajectoryParams["set_joint_trajectory"]);
            }
            else if (poseTrajectoryParams.contains("topicName"))
            {
                trajectoryActionName = HooksUtils::PluginParser::LastOnPath(poseTrajectoryParams["topicName"]);
            }

            if (HooksUtils::CreateComponent<JointsTrajectoryComponent>(entity, trajectoryActionName))
            {
                return AZ::Success();
            }
            else
            {
                return AZ::Failure(AZStd::string("Failed to create ROS2 Joint Pose Trajectory"));
            }
            

        };

        return importerHook;
    }
} // namespace ROS2::SDFormat