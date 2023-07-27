/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "SensorPluginImporterHook.h"

#include <Camera/ROS2CameraSensorEditorComponent.h>

#include <sdf/Camera.hh>

namespace ROS2::SDFormat
{
    SensorPluginImporterHook GetHook::ROS2CameraSensorComponent()
    {
        SensorPluginImporterHook importerHook;
        importerHook.m_sensorTypes =
            AZStd::unordered_set<sdf::SensorType>{ sdf::SensorType::CAMERA, sdf::SensorType::DEPTH_CAMERA, sdf::SensorType::RGBD_CAMERA };
        importerHook.m_sensorOptions =
            AZStd::unordered_set<AZStd::string>{ ">update_rate", ">camera>horizontal_fov", ">camera>image>width", ">camera>image>height" };
        importerHook.m_pluginNames = AZStd::unordered_set<AZStd::string>{ "libgazebo_ros_openni_kinect.so",
                                                                          "libgazebo_ros_camera.so",
                                                                          "libgazebo_ros_depth_camera.so" };
        importerHook.m_pluginOptions = AZStd::unordered_set<AZStd::string>{};
        importerHook.m_sdfSensorToComponentCallback = [](AZ::Entity& entity, const sdf::Sensor& sdfSensor)
        {
            auto* cameraSensor = sdfSensor.CameraSensor();
            AZ_Assert(cameraSensor, "sensor is not sdf::SensorType::CAMERA");
            sdf::ElementPtr element = cameraSensor->Element();

            CameraSensorConfiguration cameraConfiguration;
            SensorConfiguration sensorConfiguration;

            sensorConfiguration.m_frequency = sdfSensor.UpdateRate();
            // TODO: set remaining configuration in SensorConfiguration

            cameraConfiguration.m_depthCamera = cameraSensor->HasDepthCamera();
            cameraConfiguration.m_colorCamera = (sdfSensor.Type() != sdf::SensorType::DEPTH_CAMERA) ? true : false;
            cameraConfiguration.m_width = cameraSensor->ImageWidth();
            cameraConfiguration.m_height = cameraSensor->ImageHeight();
            cameraConfiguration.m_verticalFieldOfViewDeg =
                cameraSensor->HorizontalFov().Degree() * (cameraConfiguration.m_height / cameraConfiguration.m_width);

            // TODO: add distortion parameters, far/near clip after PR#419, ROS2 topic, image format, and more
            // TODO: create required services (ROS2FrameComponent)
            // TODO: read plugin's parameters (if the current plugin is supported)
            // TODO: generate log about not supported plugins and not supported options
            // TODO: use proper constructor after PR#433 is merged
            // entity.CreateComponent<ROS2CameraSensorEditorComponent>(sensorConfiguration, cameraConfiguration);
            entity.CreateComponent<ROS2CameraSensorEditorComponent>();
        };

        return importerHook;
    }
} // namespace ROS2::SDFormat
