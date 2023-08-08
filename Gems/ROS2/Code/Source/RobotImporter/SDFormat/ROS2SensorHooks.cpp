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
#include <Imu/ROS2ImuSensorComponent.h>
#include <ROS2/Frame/ROS2FrameComponent.h>
#include <RobotImporter/Utils/RobotImporterUtils.h>
#include <Source/EditorStaticRigidBodyComponent.h>

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
            if (!sdfSensor.NavSatSensor())
            {
                return AZ::Failure(AZStd::string("Failed to read parsed SDFormat data of %s NavSat sensor", sdfSensor.Name().c_str()));
            }

            SensorConfiguration sensorConfiguration;
            sensorConfiguration.m_frequency = sdfSensor.UpdateRate();
            Internal::AddTopicConfiguration(sensorConfiguration, "gnss", GNSSConstants::GNSSMessageType, GNSSConstants::GNSSDataConfig);

            if (entity.CreateComponent<ROS2GNSSSensorComponent>(sensorConfiguration, GNSSSensorConfiguration()))
            {
                return AZ::Success();
            }
            else
            {
                return AZ::Failure(AZStd::string("Failed to create ROS2 GNSS Sensor component"));
            }
        };

        return importerHook;
    }

    SensorImporterHook ROS2SensorHooks::ROS2ImuSensor()
    {
        SensorImporterHook importerHook;
        importerHook.m_sensorTypes = AZStd::unordered_set<sdf::SensorType>{ sdf::SensorType::IMU };
        importerHook.m_supportedSensorParams = AZStd::unordered_set<AZStd::string>{ ">update_rate",
                                                                                    ">imu>angular_velocity>x>noise>mean",
                                                                                    ">imu>angular_velocity>x>noise>stddev",
                                                                                    ">imu>angular_velocity>y>noise>mean",
                                                                                    ">imu>angular_velocity>y>noise>stddev",
                                                                                    ">imu>angular_velocity>z>noise>mean",
                                                                                    ">imu>angular_velocity>z>noise>stddev",
                                                                                    ">imu>linear_acceleration>x>noise>mean",
                                                                                    ">imu>linear_acceleration>x>noise>stddev",
                                                                                    ">imu>linear_acceleration>y>noise>mean",
                                                                                    ">imu>linear_acceleration>y>noise>stddev",
                                                                                    ">imu>linear_acceleration>z>noise>mean",
                                                                                    ">imu>linear_acceleration>z>noise>stddev" };
        importerHook.m_pluginNames = AZStd::unordered_set<AZStd::string>{ "libgazebo_ros_imu_sensor.so" };
        importerHook.m_supportedPluginParams = AZStd::unordered_set<AZStd::string>{};
        importerHook.m_sdfSensorToComponentCallback = [](AZ::Entity& entity,
                                                         const sdf::Sensor& sdfSensor) -> SensorImporterHook::ConvertSensorOutcome
        {
            auto* imuSensor = sdfSensor.ImuSensor();
            if (!imuSensor)
            {
                return AZ::Failure(AZStd::string("Failed to read parsed SDFormat data of %s imu sensor", sdfSensor.Name().c_str()));
            }

            ImuSensorConfiguration imuConfiguration;
            const auto& angVelXNoise = imuSensor->AngularVelocityXNoise();
            const auto& angVelYNoise = imuSensor->AngularVelocityYNoise();
            const auto& angVelZNoise = imuSensor->AngularVelocityZNoise();
            if (angVelXNoise.Type() == sdf::NoiseType::GAUSSIAN && angVelYNoise.Type() == sdf::NoiseType::GAUSSIAN &&
                angVelZNoise.Type() == sdf::NoiseType::GAUSSIAN)
            {
                if (angVelXNoise.Mean() == 0.0 && angVelYNoise.Mean() == 0.0 && angVelZNoise.Mean() == 0.0)
                {
                    imuConfiguration.m_angularVelocityVariance = AZ::Vector3(
                        static_cast<float>(angVelXNoise.StdDev() * angVelXNoise.StdDev()),
                        static_cast<float>(angVelYNoise.StdDev() * angVelYNoise.StdDev()),
                        static_cast<float>(angVelZNoise.StdDev() * angVelZNoise.StdDev()));
                }
            }

            const auto& linAccXNoise = imuSensor->LinearAccelerationXNoise();
            const auto& linAccYNoise = imuSensor->LinearAccelerationYNoise();
            const auto& linAccZNoise = imuSensor->LinearAccelerationZNoise();
            if (linAccXNoise.Type() == sdf::NoiseType::GAUSSIAN && linAccYNoise.Type() == sdf::NoiseType::GAUSSIAN &&
                linAccZNoise.Type() == sdf::NoiseType::GAUSSIAN)
            {
                if (linAccXNoise.Mean() == 0.0 && linAccYNoise.Mean() == 0.0 && linAccZNoise.Mean() == 0.0)
                {
                    imuConfiguration.m_linearAccelerationVariance = AZ::Vector3(
                        static_cast<float>(linAccXNoise.StdDev() * linAccXNoise.StdDev()),
                        static_cast<float>(linAccYNoise.StdDev() * linAccYNoise.StdDev()),
                        static_cast<float>(linAccZNoise.StdDev() * linAccZNoise.StdDev()));
                }
            }

            SensorConfiguration sensorConfiguration;
            sensorConfiguration.m_frequency = sdfSensor.UpdateRate();
            Internal::AddTopicConfiguration(sensorConfiguration, "imu", ImuConstants::ImuMessageType, ImuConstants::ImuDataConfig);

            if (entity.CreateComponent<PhysX::EditorStaticRigidBodyComponent>() && entity.CreateComponent<ROS2FrameComponent>() &&
                entity.CreateComponent<ROS2ImuSensorComponent>(sensorConfiguration, imuConfiguration))
            {
                return AZ::Success();
            }
            else
            {
                return AZ::Failure(AZStd::string("Failed to create ROS2 Imu Sensor component"));
            }
        };

        return importerHook;
    }

} // namespace ROS2::SDFormat
