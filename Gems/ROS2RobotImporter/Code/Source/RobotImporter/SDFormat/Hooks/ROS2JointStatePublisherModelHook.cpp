/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <AzCore/std/containers/vector.h>
#include <AzCore/std/string/string.h>
#include <ROS2/Communication/PublisherConfiguration.h>
#include <ROS2/ROS2EditorBus.h>
#include <ROS2Controllers/ROS2ControllersEditorBus.h>
#include <RobotImporter/SDFormat/ROS2ModelPluginHooks.h>
#include <RobotImporter/SDFormat/ROS2SDFormatHooksUtils.h>
#include <RobotImporter/Utils/RobotImporterUtils.h>
#include <Source/EditorArticulationLinkComponent.h>

namespace ROS2RobotImporter::SDFormat
{
    ModelPluginImporterHook ROS2ModelPluginHooks::ROS2JointStatePublisherModel()
    {
        ModelPluginImporterHook importerHook;
        importerHook.m_pluginNames = AZStd::unordered_set<AZStd::string>{ "libgazebo_ros_joint_state_publisher.so" };
        importerHook.m_supportedPluginParams =
            AZStd::unordered_set<AZStd::string>{ ">updateRate", ">update_rate", ">ros>argument", ">ros>remapping", ">topicName" };

        importerHook.m_sdfPluginToComponentCallback =
            [](AZ::Entity& entity, const sdf::Plugin& sdfPlugin, const sdf::Model& sdfModel, const CreatedEntitiesMap& createdEntities)
            -> ModelPluginImporterHook::ConvertPluginOutcome
        {
            const auto statePublisherParams = HooksUtils::GetPluginParams({ sdfPlugin });

            // configure publisher
            ROS2::PublisherConfiguration publisherConfiguration;
            publisherConfiguration.m_frequency = HooksUtils::GetFrequency(statePublisherParams);
            publisherConfiguration.m_topicConfiguration.m_type = "sensor_msgs::msg::JointState";

            const static AZStd::vector<AZStd::string> topicParamNames = { "topicName", "joint_states" };
            publisherConfiguration.m_topicConfiguration.m_topic =
                HooksUtils::ValueOfAny(statePublisherParams, topicParamNames, "joint_states");

            // create controllerComponent based on model joints/articulations
            auto* ros2interface = ROS2::ROS2EditorInterface::Get();
            AZ_Assert(ros2interface, "ROS2EditorInterface not available in ROS2ImuSensorHook");
            auto* controllersInterface = ROS2Controllers::ROS2ControllersEditorInterface::Get();
            AZ_Assert(controllersInterface, "ROS2ControllersEditorInterface not available in ROS2JointStatePublisherModelPluginHook");
            if (ros2interface && controllersInterface)
            {
                auto* ros2FrameComponent = ros2interface->CreateROS2FrameEditorComponent(entity, ROS2::ROS2FrameConfiguration());
                entity.FindComponent<PhysX::EditorArticulationLinkComponent>()
                    ? controllersInterface->CreateJointsArticulationControllerComponent(entity)
                    : controllersInterface->CreateJointsPIDControllerComponent(entity);
                auto* jointComponent = controllersInterface->CreateJointsManipulationEditorComponent(entity, publisherConfiguration);
                if (ros2FrameComponent && jointComponent)
                {
                    return AZ::Success();
                }
            }

            return AZ::Failure(AZStd::string("Failed to create ROS 2 Joint State Publisher Component"));
        };

        return importerHook;
    }
} // namespace ROS2RobotImporter::SDFormat
