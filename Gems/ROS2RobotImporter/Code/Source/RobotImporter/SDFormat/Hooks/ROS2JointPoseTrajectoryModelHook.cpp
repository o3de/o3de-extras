/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <Manipulation/Controllers/JointsArticulationControllerComponent.h>
#include <Manipulation/Controllers/JointsPIDControllerComponent.h>
#include <Manipulation/JointsManipulationEditorComponent.h>
#include <Manipulation/JointsTrajectoryComponent.h>
#include <ROS2/Frame/ROS2FrameEditorComponent.h>
#include <RobotImporter/SDFormat/ROS2ModelPluginHooks.h>
#include <RobotImporter/SDFormat/ROS2SDFormatHooksUtils.h>
#include <Source/EditorArticulationLinkComponent.h>

namespace ROS2RobotImporter::SDFormat
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
            const auto poseTrajectoryParams = HooksUtils::GetPluginParams({ sdfPlugin });

            // configure JointsManipulationEditorComponent publisher
            PublisherConfiguration publisherConfiguration;
            publisherConfiguration.m_frequency = HooksUtils::GetFrequency(poseTrajectoryParams);
            publisherConfiguration.m_topicConfiguration.m_type = "sensor_msgs::msg::JointState";
            publisherConfiguration.m_topicConfiguration.m_topic = "joint_states";

            // configure trajectory action name
            const static AZStd::vector<AZStd::string> trajectoryTopicParamNames = { "set_joint_trajectory", "topicName" };
            const AZStd::string trajectoryActionName =
                HooksUtils::ValueOfAny(poseTrajectoryParams, trajectoryTopicParamNames, "arm_controller/follow_joint_trajectory");

            // add required components
            HooksUtils::CreateComponent<ROS2::ROS2FrameEditorComponent>(entity);

            // create controllerComponent based on model joints/articulations
            entity.FindComponent<PhysX::EditorArticulationLinkComponent>()
                ? HooksUtils::CreateComponent<ROS2Controllers::JointsArticulationControllerComponent>(entity)
                : HooksUtils::CreateComponent<ROS2Controllers::JointsPIDControllerComponent>(entity);

            HooksUtils::CreateComponent<ROS2Controllers::JointsManipulationEditorComponent>(entity, publisherConfiguration);

            if (HooksUtils::CreateComponent<ROS2Controllers::JointsTrajectoryComponent>(entity, trajectoryActionName))
            {
                return AZ::Success();
            }
            else
            {
                return AZ::Failure(AZStd::string("Failed to create ROS 2 Joint Pose Trajectory Component"));
            }
        };

        return importerHook;
    }
} // namespace ROS2RobotImporter::SDFormat
