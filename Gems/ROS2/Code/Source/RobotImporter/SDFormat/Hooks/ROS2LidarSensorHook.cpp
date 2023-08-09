/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <Lidar/ROS2LidarSensorComponent.h>
#include <ROS2/Frame/ROS2FrameComponent.h>
#include <RobotImporter/SDFormat/ROS2SensorHooks.h>

#include <sdf/Lidar.hh>
#include <sdf/Sensor.hh>

namespace ROS2::SDFormat
{
    SensorImporterHook ROS2SensorHooks::ROS2LidarSensor()
    {
        SensorImporterHook importerHook;
        importerHook.m_sensorTypes = AZStd::unordered_set<sdf::SensorType>{ sdf::SensorType::LIDAR };
        importerHook.m_supportedSensorParams = AZStd::unordered_set<AZStd::string>{
            ">update_rate",
            ">lidar>scan>horizontal>samples",
            ">lidar>scan>horizontal>min_angle",
            ">lidar>scan>horizontal>max_angle",
            ">lidar>scan>vertical>samples",
            ">lidar>scan>vertical>min_angle",
            ">lidar>scan>vertical>max_angle",
            ">lidar>range>min",
            ">lidar>range>max",
            // Gazebo-classic
            ">ray>scan>horizontal>samples",
            ">ray>scan>horizontal>min_angle",
            ">ray>scan>horizontal>max_angle",
            ">ray>scan>vertical>samples",
            ">ray>scan>vertical>min_angle",
            ">ray>scan>vertical>max_angle",
            ">ray>range>min",
            ">ray>range>max",
        };
        importerHook.m_pluginNames = AZStd::unordered_set<AZStd::string>{ "libgazebo_ros_ray_sensor.so", "libgazebo_ros_laser.so" };
        importerHook.m_supportedPluginParams = AZStd::unordered_set<AZStd::string>{};
        importerHook.m_sdfSensorToComponentCallback = [](AZ::Entity& entity,
                                                         const sdf::Sensor& sdfSensor) -> SensorImporterHook::ConvertSensorOutcome
        {
            auto* lidarSensor = sdfSensor.LidarSensor();
            if (!lidarSensor)
            {
                return AZ::Failure(AZStd::string("Failed to read parsed SDFormat data of %s Lidar sensor", sdfSensor.Name().c_str()));
            }

            SensorConfiguration sensorConfiguration;
            sensorConfiguration.m_frequency = sdfSensor.UpdateRate();
            const AZStd::string messageType = "sensor_msgs::msg::PointCloud2";
            Utils::AddTopicConfiguration(sensorConfiguration, "pc", messageType, messageType);

            LidarSensorConfiguration lidarConfiguration;
            lidarConfiguration.m_lidarParameters.m_model = LidarTemplate::LidarModel::Custom3DLidar;
            lidarConfiguration.m_lidarParameters.m_name = AZStd::string(sdfSensor.Name().c_str());
            lidarConfiguration.m_lidarParameters.m_minHAngle = lidarSensor->HorizontalScanMinAngle().Degree();
            lidarConfiguration.m_lidarParameters.m_maxHAngle = lidarSensor->HorizontalScanMaxAngle().Degree();
            lidarConfiguration.m_lidarParameters.m_minVAngle = lidarSensor->VerticalScanMinAngle().Degree();
            lidarConfiguration.m_lidarParameters.m_maxVAngle = lidarSensor->VerticalScanMaxAngle().Degree();
            lidarConfiguration.m_lidarParameters.m_layers = lidarSensor->HorizontalScanSamples();
            lidarConfiguration.m_lidarParameters.m_numberOfIncrements = lidarSensor->VerticalScanSamples();
            lidarConfiguration.m_lidarParameters.m_minRange = lidarSensor->RangeMin();
            lidarConfiguration.m_lidarParameters.m_maxRange = lidarSensor->RangeMax();

            if (entity.CreateComponent<ROS2FrameComponent>() &&
                entity.CreateComponent<ROS2LidarSensorComponent>(sensorConfiguration, lidarConfiguration))
            {
                return AZ::Success();
            }
            else
            {
                return AZ::Failure(AZStd::string("Failed to create ROS2 Lidar Sensor component"));
            }
        };

        return importerHook;
    }

} // namespace ROS2::SDFormat
