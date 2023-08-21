/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <Camera/CameraConstants.h>
#include <Camera/ROS2CameraSensorEditorComponent.h>
#include <ROS2/Frame/ROS2FrameComponent.h>
#include <ROS2/ROS2GemUtilities.h>
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
            if (!cameraSensor)
            {
                return AZ::Failure(AZStd::string("Failed to read parsed SDFormat data of %s camera sensor", sdfSensor.Name().c_str()));
            }

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
            { // COLOR_CAMERA and RGBD_CAMERA
                Utils::AddTopicConfiguration(
                    sensorConfiguration, "camera_image_color", CameraConstants::ImageMessageType, CameraConstants::ColorImageConfig);
                Utils::AddTopicConfiguration(
                    sensorConfiguration, "color_camera_info", CameraConstants::CameraInfoMessageType, CameraConstants::ColorInfoConfig);
            }
            if (sdfSensor.Type() != sdf::SensorType::CAMERA)
            { // DEPTH_CAMERA and RGBD_CAMERA
                Utils::AddTopicConfiguration(
                    sensorConfiguration, "camera_image_depth", CameraConstants::ImageMessageType, CameraConstants::DepthImageConfig);
                Utils::AddTopicConfiguration(
                    sensorConfiguration, "depth_camera_info", CameraConstants::CameraInfoMessageType, CameraConstants::DepthInfoConfig);
            }

            const auto& entityId = entity.GetId();
            if (!ROS2::Utils::CreateComponent(entityId, ROS2FrameComponent::TYPEINFO_Uuid()))
            {
                return AZ::Failure(AZStd::string("Failed to create ROS2FrameComponent required for ROS2 Camera Sensor component"));
            }
            const auto componentId = ROS2::Utils::CreateComponent(entityId, ROS2CameraSensorEditorComponent::TYPEINFO_Uuid());
            if (componentId)
            {
                auto* component = ROS2::Utils::GetGameOrEditorComponent<ROS2CameraSensorEditorComponent>(&entity);
                component->SetSensorConfiguration(sensorConfiguration);
                component->SetCameraSensorConfiguration(cameraConfiguration);
            }
            else
            {
                return AZ::Failure(AZStd::string("Failed to create ROS2 Camera Sensor component"));
            }

            return AZ::Success();
        };

        return importerHook;
    }
} // namespace ROS2::SDFormat
