/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <Camera/CameraConstants.h>
#include <Camera/ROS2CameraSensorEditorComponent.h>
#include <ROS2/Frame/ROS2FrameEditorComponent.h>
#include <RobotImporter/SDFormat/ROS2SDFormatHooksUtils.h>
#include <RobotImporter/SDFormat/ROS2SensorHooks.h>

#include <sdf/Camera.hh>
#include <sdf/Sensor.hh>

namespace ROS2::SDFormat
{
    SensorImporterHook ROS2SensorHooks::ROS2CameraSensor()
    {
        SensorImporterHook importerHook;
        importerHook.m_sensorTypes =
            AZStd::unordered_set<sdf::SensorType>{ sdf::SensorType::CAMERA, sdf::SensorType::DEPTH_CAMERA, sdf::SensorType::RGBD_CAMERA };
        importerHook.m_supportedSensorParams = AZStd::unordered_set<AZStd::string>{
            ">pose",           ">update_rate", ">camera>horizontal_fov", ">camera>image>width", ">camera>image>height", ">camera>clip>near",
            ">camera>clip>far"
        };
        importerHook.m_pluginNames = AZStd::unordered_set<AZStd::string>{ "libgazebo_ros_camera.so",
                                                                          "libgazebo_ros_depth_camera.so",
                                                                          "libgazebo_ros_openni_kinect.so" };
        importerHook.m_supportedPluginParams = AZStd::unordered_set<AZStd::string>{};
        importerHook.m_sdfSensorToComponentCallback = [](AZ::Entity& entity,
                                                         const sdf::Sensor& sdfSensor) -> SensorImporterHook::ConvertSensorOutcome
        {
            auto* cameraSensor = sdfSensor.CameraSensor();
            if (!cameraSensor)
            {
                return AZ::Failure(AZStd::string("Failed to read parsed SDFormat data of %s camera sensor", sdfSensor.Name().c_str()));
            }

            CameraSensorConfiguration cameraConfiguration;
            cameraConfiguration.m_depthCamera = cameraSensor->HasDepthCamera();
            cameraConfiguration.m_colorCamera = (sdfSensor.Type() != sdf::SensorType::DEPTH_CAMERA) ? true : false;
            cameraConfiguration.m_width = cameraSensor->ImageWidth();
            cameraConfiguration.m_height = cameraSensor->ImageHeight();
            if (cameraConfiguration.m_width != 0)
            {
                double aspectRatio = static_cast<double>(cameraConfiguration.m_height) / cameraConfiguration.m_width;
                cameraConfiguration.m_verticalFieldOfViewDeg =
                    AZ::RadToDeg(2.0 * AZStd::atan(AZStd::tan(cameraSensor->HorizontalFov().Radian() / 2.0) * aspectRatio));
            }
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
            { // COLOR_CAMERA and RGBD_CAMERA
                HooksUtils::AddTopicConfiguration(
                    sensorConfiguration, "camera_image_color", CameraConstants::ImageMessageType, CameraConstants::ColorImageConfig);
                HooksUtils::AddTopicConfiguration(
                    sensorConfiguration, "color_camera_info", CameraConstants::CameraInfoMessageType, CameraConstants::ColorInfoConfig);
            }
            if (sdfSensor.Type() != sdf::SensorType::CAMERA)
            { // DEPTH_CAMERA and RGBD_CAMERA
                HooksUtils::AddTopicConfiguration(
                    sensorConfiguration, "camera_image_depth", CameraConstants::ImageMessageType, CameraConstants::DepthImageConfig);
                HooksUtils::AddTopicConfiguration(
                    sensorConfiguration, "depth_camera_info", CameraConstants::CameraInfoMessageType, CameraConstants::DepthInfoConfig);
            }

            // Create required components
            HooksUtils::CreateComponent<ROS2FrameEditorComponent>(entity);

            // Create Camera component
            if (HooksUtils::CreateComponent<ROS2CameraSensorEditorComponent>(entity, sensorConfiguration, cameraConfiguration))
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
