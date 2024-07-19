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
        importerHook.m_supportedPluginParams = AZStd::unordered_set<AZStd::string>{
            ">imageTopicName", ">cameraInfoTopicName", ">depthImageTopicName", ">depthImageCameraInfoTopicName",
            ">ros>remapping", ">ros>argument"
        };
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

            const auto cameraPlugins = sdfSensor.Plugins();
            HooksUtils::PluginParams cameraPluginParams = cameraPlugins.empty() ? HooksUtils::PluginParams() : HooksUtils::GetPluginParams(cameraPlugins[0]);

            // add depth_camera for plugins kinnect and depth_camera
            // check only the 1st plugin as it's the only one considered afterwards
            if (!cameraPlugins.empty() &&
                cameraPlugins[0].Filename() == "libgazebo_ros_depth_camera.so" ||
                cameraPlugins[0].Filename() == "libgazebo_ros_openni_kinect.so")
            {
                cameraConfiguration.m_depthCamera = true;
            }

            if (sdfSensor.Type() != sdf::SensorType::DEPTH_CAMERA)
            { // COLOR_CAMERA and RGBD_CAMERA
                AZStd::string imageColor = "camera_image_color", colorInfo = "camera_color_info";

                if (cameraPluginParams.contains("image_raw")) {
                    imageColor = cameraPluginParams["image_raw"];
                }
                else if (cameraPluginParams.contains("imageTopicName")) {
                    imageColor = HooksUtils::PluginParser::LastOnPath(cameraPluginParams["imageTopicName"]);
                }

                if (cameraPluginParams.contains("camera_info")) {
                    colorInfo = cameraPluginParams["camera_info"];
                }
                else if (cameraPluginParams.contains("cameraInfoTopicName")) {
                    colorInfo = HooksUtils::PluginParser::LastOnPath(cameraPluginParams["cameraInfoTopicName"]);
                }

                HooksUtils::AddTopicConfiguration(
                    sensorConfiguration, imageColor, CameraConstants::ImageMessageType, CameraConstants::ColorImageConfig);
                HooksUtils::AddTopicConfiguration(
                    sensorConfiguration, colorInfo, CameraConstants::CameraInfoMessageType, CameraConstants::ColorInfoConfig);
            }
            if (cameraConfiguration.m_depthCamera)
            { // DEPTH_CAMERA and RGBD_CAMERA
                AZStd::string imageDepth = "camera_image_depth", depthInfo = "depth_camera_info";

                if (cameraPluginParams.contains("image_depth")) {
                    imageDepth = cameraPluginParams["image_depth"];
                }
                else if (cameraPluginParams.contains("depthImageTopicName")) {
                    imageDepth = HooksUtils::PluginParser::LastOnPath(cameraPluginParams["depthImageTopicName"]);
                }

                if (cameraPluginParams.contains("camera_info_depth")) {
                    depthInfo = cameraPluginParams["camera_info_depth"];
                }
                else if (cameraPluginParams.contains("depthImageCameraInfoTopicName")) {
                    depthInfo = HooksUtils::PluginParser::LastOnPath(cameraPluginParams["depthImageCameraInfoTopicName"]);
                }

                HooksUtils::AddTopicConfiguration(
                    sensorConfiguration, imageDepth, CameraConstants::ImageMessageType, CameraConstants::DepthImageConfig);
                HooksUtils::AddTopicConfiguration(
                    sensorConfiguration, depthInfo, CameraConstants::CameraInfoMessageType, CameraConstants::DepthInfoConfig);
            }

            // Create required components
            auto frameComponent = HooksUtils::CreateComponent<ROS2FrameEditorComponent>(entity);
            HooksUtils::ConfigureFrame(frameComponent, cameraPluginParams);

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
