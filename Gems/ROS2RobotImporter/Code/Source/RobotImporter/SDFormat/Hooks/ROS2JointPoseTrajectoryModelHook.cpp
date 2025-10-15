/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <ROS2/Communication/PublisherConfiguration.h>
#include <ROS2/ROS2EditorBus.h>
#include <ROS2Controllers/ROS2ControllersEditorBus.h>
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
            ROS2::PublisherConfiguration publisherConfiguration;
            publisherConfiguration.m_frequency = HooksUtils::GetFrequency(poseTrajectoryParams);
            publisherConfiguration.m_topicConfiguration.m_type = "sensor_msgs::msg::JointState";
            publisherConfiguration.m_topicConfiguration.m_topic = "joint_states";

            // configure trajectory action name
            const static AZStd::vector<AZStd::string> trajectoryTopicParamNames = { "set_joint_trajectory", "topicName" };
            const AZStd::string trajectoryActionName =
                HooksUtils::ValueOfAny(poseTrajectoryParams, trajectoryTopicParamNames, "arm_controller/follow_joint_trajectory");

            // create controllerComponent based on model joints/articulations
            auto* ros2interface = ROS2::ROS2EditorInterface::Get();
            AZ_Assert(ros2interface, "ROS2EditorInterface not available in ROS2ImuSensorHook");
            auto* controllersInterface = ROS2Controllers::ROS2ControllersEditorInterface::Get();
            AZ_Assert(controllersInterface, "ROS2ControllersEditorInterface not available in ROS2JointPoseTrajectoryModelPluginHook");
            if (ros2interface && controllersInterface)
            {
                auto* ros2FrameComponent = ros2interface->CreateROS2FrameEditorComponent(entity, ROS2::ROS2FrameConfiguration());
                entity.FindComponent<PhysX::EditorArticulationLinkComponent>()
                    ? controllersInterface->CreateJointsArticulationControllerComponent(entity)
                    : controllersInterface->CreateJointsPIDControllerComponent(entity);

                controllersInterface->CreateJointsManipulationEditorComponent(entity, publisherConfiguration);
                auto* trajectoryComponent = controllersInterface->CreateJointsTrajectoryComponent(entity, trajectoryActionName);
                if (ros2FrameComponent && trajectoryComponent)
                {
                    return AZ::Success();
                }
            }

            return AZ::Failure(AZStd::string("Failed to create ROS 2 Joints Trajectory Component"));
        };

        return importerHook;
    }
} // namespace ROS2RobotImporter::SDFormat
