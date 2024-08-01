/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <Manipulation/Controllers/JointsArticulationControllerComponent.h>
#include <Manipulation/JointsManipulationEditorComponent.h>
#include <Manipulation/JointsTrajectoryComponent.h>
#include <ROS2/Frame/ROS2FrameEditorComponent.h>
#include <RobotImporter/SDFormat/ROS2ModelPluginHooks.h>
#include <RobotImporter/SDFormat/ROS2SDFormatHooksUtils.h>

namespace ROS2::SDFormat
{
    ModelPluginImporterHook ROS2ModelPluginHooks::ROS2JointPoseTrajectoryModel()
    {
        ModelPluginImporterHook importerHook;
        importerHook.m_pluginNames = AZStd::unordered_set<AZStd::string>{ "libgazebo_ros_joint_pose_trajectory.so" };
        importerHook.m_supportedPluginParams =
            AZStd::unordered_set<AZStd::string>{ ">topicName", ">ros>remapping", ">ros>argument", ">updateRate", ">update_rate" };

        importerHook.m_sdfPluginToComponentCallback =
            [](AZ::Entity& entity, const sdf::Plugin& sdfPlugin, const sdf::Model& sdfModel, const CreatedEntitiesMap& createdEntities)
            -> ModelPluginImporterHook::ConvertPluginOutcome
        {
            HooksUtils::PluginParams poseTrajectoryParams = HooksUtils::GetPluginParams(sdfPlugin);

            // configure JointsManipulationEditorComponent publisher
            PublisherConfiguration publisherConfiguration;
            if (poseTrajectoryParams.contains("update_rate"))
            {
                publisherConfiguration.m_frequency = AZStd::stof(poseTrajectoryParams["update_rate"]);
            }
            else if (poseTrajectoryParams.contains("updateRate"))
            {
                publisherConfiguration.m_frequency = AZStd::stof(poseTrajectoryParams["updateRate"]);
            }
            publisherConfiguration.m_topicConfiguration.m_type = "sensor_msgs::msg::JointState";
            publisherConfiguration.m_topicConfiguration.m_topic = "joint_states";

            // configure trajectory action name
            const AZStd::string trajectoryActionName = HooksUtils::PluginParser::LastOnPath(HooksUtils::ValueOfAny(
                poseTrajectoryParams, { "set_joint_trajectory", "topicName" }, "arm_controller/follow_joint_trajectory"));

            // add required components
            HooksUtils::CreateComponent<JointsArticulationControllerComponent>(entity);
            HooksUtils::CreateComponent<ROS2FrameEditorComponent>(entity);
            HooksUtils::CreateComponent<JointsManipulationEditorComponent>(entity, publisherConfiguration);

            if (HooksUtils::CreateComponent<JointsTrajectoryComponent>(entity, trajectoryActionName))
            {
                return AZ::Success();
            }
            else
            {
                return AZ::Failure(AZStd::string("Failed to create ROS2 Joint Pose Trajectory Component"));
            }
        };

        return importerHook;
    }
} // namespace ROS2::SDFormat
