/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <ROS2/Frame/ROS2FrameEditorComponent.h>
#include <ROS2Sensors/Camera/CameraSensorConfiguration.h>
#include <ROS2Sensors/ROS2SensorsEditorBus.h>
#include <RobotImporter/SDFormat/ROS2SDFormatHooksUtils.h>
#include <RobotImporter/SDFormat/ROS2SensorHooks.h>

#include <sdf/Camera.hh>
#include <sdf/Sensor.hh>

namespace ROS2RobotImporter::SDFormat
{
    SensorImporterHook ROS2SensorHooks::ROS2CameraSensor()
    {
        SensorImporterHook importerHook;
        importerHook.m_sensorTypes =
            AZStd::unordered_set<sdf::SensorType>{ sdf::SensorType::CAMERA, sdf::SensorType::DEPTH_CAMERA, sdf::SensorType::RGBD_CAMERA };
        importerHook.m_supportedSensorParams = AZStd::unordered_set<AZStd::string>{ ">pose",
                                                                                    ">update_rate",
                                                                                    ">camera>horizontal_fov",
                                                                                    ">camera>image>width",
                                                                                    ">camera>image>height",
                                                                                    ">camera>clip>near",
                                                                                    ">camera>clip>far",
                                                                                    ">visualize" };
        importerHook.m_pluginNames = AZStd::unordered_set<AZStd::string>{ "libgazebo_ros_camera.so",
                                                                          "libgazebo_ros_depth_camera.so",
                                                                          "libgazebo_ros_openni_kinect.so" };
        importerHook.m_supportedPluginParams = AZStd::unordered_set<AZStd::string>{
            ">imageTopicName", ">cameraInfoTopicName", ">depthImageTopicName", ">depthImageCameraInfoTopicName",
            ">ros>remapping",  ">ros>argument",        ">ros>frame_name",      ">ros>namespace",
            ">updateRate",     ">frameName",           ">robotNamespace"
        };
        importerHook.m_sdfSensorToComponentCallback = [](AZ::Entity& entity,
                                                         const sdf::Sensor& sdfSensor) -> SensorImporterHook::ConvertSensorOutcome
        {
            auto* cameraSensor = sdfSensor.CameraSensor();
            if (!cameraSensor)
            {
                return AZ::Failure(AZStd::string("Failed to read parsed SDFormat data of %s camera sensor", sdfSensor.Name().c_str()));
            }

            // Get camera sensor configuration
            ROS2Sensors::CameraSensorConfiguration cameraConfiguration;
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

            const auto cameraPluginParams = HooksUtils::GetPluginParams(sdfSensor.Plugins());
            const auto element = sdfSensor.Element();

            // Get base sensor configuration
            ROS2::SensorConfiguration sensorConfiguration;
            element->Get<bool>("visualize", sensorConfiguration.m_visualize, false);
            element->Get<float>("update_rate", sensorConfiguration.m_frequency, 10.0f);

            if (sdfSensor.Type() != sdf::SensorType::DEPTH_CAMERA)
            { // COLOR_CAMERA and RGBD_CAMERA
                const static AZStd::vector<AZStd::string> rawImageParamNames = { "image_raw", "imageTopicName" };
                const static AZStd::vector<AZStd::string> rawInfoParamNames = { "camera_info", "cameraInfoTopicName" };

                const AZStd::string imageColor = HooksUtils::ValueOfAny(cameraPluginParams, rawImageParamNames, "camera_image_color");
                const AZStd::string colorInfo = HooksUtils::ValueOfAny(cameraPluginParams, rawInfoParamNames, "camera_color_info");

                HooksUtils::AddTopicConfiguration(
                    sensorConfiguration,
                    imageColor,
                    ROS2Sensors::CameraConstants::ImageMessageType,
                    ROS2Sensors::CameraConstants::ColorImageConfig);
                HooksUtils::AddTopicConfiguration(
                    sensorConfiguration,
                    colorInfo,
                    ROS2Sensors::CameraConstants::CameraInfoMessageType,
                    ROS2Sensors::CameraConstants::ColorInfoConfig);
            }
            if (sdfSensor.Type() != sdf::SensorType::CAMERA)
            { // DEPTH_CAMERA and RGBD_CAMERA
                const static AZStd::vector<AZStd::string> depthImageParamNames = { "image_depth", "depthImageTopicName" };
                const static AZStd::vector<AZStd::string> depthInfoParamNames = { "camera_info_depth", "depthImageCameraInfoTopicName" };

                const AZStd::string imageDepth = HooksUtils::ValueOfAny(cameraPluginParams, depthImageParamNames, "camera_image_depth");
                const AZStd::string depthInfo = HooksUtils::ValueOfAny(cameraPluginParams, depthInfoParamNames, "depth_camera_info");

                HooksUtils::AddTopicConfiguration(
                    sensorConfiguration,
                    imageDepth,
                    ROS2Sensors::CameraConstants::ImageMessageType,
                    ROS2Sensors::CameraConstants::DepthImageConfig);
                HooksUtils::AddTopicConfiguration(
                    sensorConfiguration,
                    depthInfo,
                    ROS2Sensors::CameraConstants::CameraInfoMessageType,
                    ROS2Sensors::CameraConstants::DepthInfoConfig);
            }

            // Get frame configuration
            const auto frameConfiguration = HooksUtils::GetFrameConfiguration(cameraPluginParams);

            // Create required components
            HooksUtils::CreateComponent<ROS2::ROS2FrameEditorComponent>(entity, frameConfiguration);

            // Create Camera component
            auto interface = ROS2Sensors::ROS2SensorsEditorInterface::Get();
            AZ_Warning("ROS2RobotImporter", interface, "ROS2SensorsInterface is not available. Cannot create Camera sensor component.");
            if (interface)
            {
                if (auto* sensor = interface->CreateROS2CameraSensorComponent(entity, sensorConfiguration, cameraConfiguration))
                {
                    return AZ::Success();
                }
            }

            return AZ::Failure(AZStd::string("Failed to create ROS 2 Camera Sensor component"));
        };

        return importerHook;
    }
} // namespace ROS2RobotImporter::SDFormat
