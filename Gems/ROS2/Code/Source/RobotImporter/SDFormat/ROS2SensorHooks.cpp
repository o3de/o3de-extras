/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ROS2SensorHooks.h"

#include <Camera/CameraConstants.h>
#include <Camera/ROS2CameraSensorEditorComponent.h>
#include <GNSS/ROS2GNSSSensorComponent.h>
#include <ROS2/Frame/ROS2FrameComponent.h>
#include <RobotImporter/Utils/RobotImporterUtils.h>

#include <sdf/Camera.hh>
#include <sdf/NavSat.hh>

namespace ROS2::SDFormat
{
    namespace Internal
    {
        void AddTopicConfiguration(
            SensorConfiguration& sensorConfig,
            const AZStd::string& topic,
            const AZStd::string& messageType,
            const AZStd::string& configName)
        {
            TopicConfiguration config;
            config.m_topic = topic;
            config.m_type = messageType;
            sensorConfig.m_publishersConfigurations.insert(AZStd::make_pair(configName, config));
        }
    } // namespace Internal

    SensorImporterHook ROS2SensorHooks::ROS2CameraSensor()
    {
        SensorImporterHook importerHook;
        importerHook.m_sensorTypes =
            AZStd::unordered_set<sdf::SensorType>{ sdf::SensorType::CAMERA, sdf::SensorType::DEPTH_CAMERA, sdf::SensorType::RGBD_CAMERA };
        importerHook.m_supportedSensorParams =
            AZStd::unordered_set<AZStd::string>{ ">update_rate",         ">camera>horizontal_fov", ">camera>image>width",
                                                 ">camera>image>height", ">camera>clip>near",      ">camera>clip>far" };
        importerHook.m_pluginNames = AZStd::unordered_set<AZStd::string>{ "libgazebo_ros_camera.so",
                                                                          "libgazebo_ros_depth_camera.so",
                                                                          "libgazebo_ros_openni_kinect.so" };
        importerHook.m_supportedPluginParams = AZStd::unordered_set<AZStd::string>{};
        importerHook.m_sdfSensorToComponentCallback = [](AZ::Entity& entity,
                                                         const sdf::Sensor& sdfSensor) -> SensorImporterHook::ConvertSensorOutcome
        {
            auto* cameraSensor = sdfSensor.CameraSensor();

            CameraSensorConfiguration cameraConfiguration;
            cameraConfiguration.m_depthCamera = cameraSensor->HasDepthCamera();
            cameraConfiguration.m_colorCamera = (sdfSensor.Type() != sdf::SensorType::DEPTH_CAMERA) ? true : false;
            cameraConfiguration.m_width = cameraSensor->ImageWidth();
            cameraConfiguration.m_height = cameraSensor->ImageHeight();
            cameraConfiguration.m_verticalFieldOfViewDeg =
                cameraSensor->HorizontalFov().Degree() * (cameraConfiguration.m_height / cameraConfiguration.m_width);
            if (sdfSensor.Type() != sdf::SensorType::DEPTH_CAMERA)
            {
                cameraConfiguration.m_nearClipDistance = static_cast<float>(cameraSensor->NearClip());
                cameraConfiguration.m_farClipDistance = static_cast<float>(cameraSensor->FarClip());
            }
            else
            {
                cameraConfiguration.m_nearClipDistance = static_cast<float>(cameraSensor->DepthNearClip());
                cameraConfiguration.m_farClipDistance = static_cast<float>(cameraSensor->DepthFarClip());
            }

            SensorConfiguration sensorConfiguration;
            sensorConfiguration.m_frequency = sdfSensor.UpdateRate();
            if (sdfSensor.Type() != sdf::SensorType::DEPTH_CAMERA)
            {
                Internal::AddTopicConfiguration(
                    sensorConfiguration, "camera_image_color", CameraConstants::ImageMessageType, CameraConstants::ColorImageConfig);
                Internal::AddTopicConfiguration(
                    sensorConfiguration, "color_camera_info", CameraConstants::CameraInfoMessageType, CameraConstants::ColorInfoConfig);
            }
            if (sdfSensor.Type() != sdf::SensorType::CAMERA)
            {
                Internal::AddTopicConfiguration(
                    sensorConfiguration, "camera_image_depth", CameraConstants::ImageMessageType, CameraConstants::DepthImageConfig);
                Internal::AddTopicConfiguration(
                    sensorConfiguration, "depth_camera_info", CameraConstants::CameraInfoMessageType, CameraConstants::DepthInfoConfig);
            }

            if (entity.CreateComponent<ROS2FrameComponent>() &&
                entity.CreateComponent<ROS2CameraSensorEditorComponent>(sensorConfiguration, cameraConfiguration))
            {
                return AZ::Success();
            }
            else
            {
                return AZ::Failure(AZStd::string("Failed to create ROS2 Camera Sensor component"));
            }
        };

        return importerHook;
    }

    SensorImporterHook ROS2SensorHooks::ROS2GNSSSensor()
    {
        SensorImporterHook importerHook;
        importerHook.m_sensorTypes = AZStd::unordered_set<sdf::SensorType>{ sdf::SensorType::NAVSAT };
        importerHook.m_supportedSensorParams = AZStd::unordered_set<AZStd::string>{ ">update_rate" };
        importerHook.m_pluginNames = AZStd::unordered_set<AZStd::string>{ "libgazebo_ros_gps_sensor.so" };
        importerHook.m_supportedPluginParams = AZStd::unordered_set<AZStd::string>{};
        importerHook.m_sdfSensorToComponentCallback = [](AZ::Entity& entity,
                                                         const sdf::Sensor& sdfSensor) -> SensorImporterHook::ConvertSensorOutcome
        {
            SensorConfiguration sensorConfiguration;
            sensorConfiguration.m_frequency = sdfSensor.UpdateRate();
            Internal::AddTopicConfiguration(sensorConfiguration, "gnss", GNSSConstants::GNSSMessageType, GNSSConstants::GNSSDataConfig);

            if (entity.CreateComponent<ROS2GNSSSensorComponent>(sensorConfiguration, GNSSSensorConfiguration()))
            {
                return AZ::Success();
            }
            else
            {
                return AZ::Failure(AZStd::string("Failed to create ROS2 Camera Sensor component"));
            }
        };

        return importerHook;
    }

} // namespace ROS2::SDFormat
