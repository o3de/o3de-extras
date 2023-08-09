/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/std/string/string.h>
#include <ROS2/RobotImporter/SDFormatSensorImporterHook.h>
#include <ROS2/Sensor/SensorConfiguration.h>

namespace ROS2::SDFormat
{
    namespace ROS2SensorHooks
    {
        namespace Utils
        {
            //! Add a ROS2 topic configuration to sensor parameters.
            //! @param sensorConfig sensor's configuration which hosts multiple topic configurations
            //! @param topic ROS2 topic name
            //! @param messageType ROS2 message type
            //! @param configName name under which topic configuration is stored in sensor's configuration
            void AddTopicConfiguration(
                SensorConfiguration& sensorConfig,
                const AZStd::string& topic,
                const AZStd::string& messageType,
                const AZStd::string& configName);
        } // namespace Utils

        //! Retrieve a sensor importer hook which is used to map SDFormat sensor of type camera, depth or rgbd into O3DE
        //! ROS2CameraSensorComponent
        SensorImporterHook ROS2CameraSensor();

        //! Retrieve a sensor importer hook which is used to map SDFormat sensor of type navsat into O3DE ROS2GNSSSensorComponent
        SensorImporterHook ROS2GNSSSensor();

        //! Retrieve a sensor importer hook which is used to map SDFormat sensor of type imu into O3DE ROS2ImuSensorComponent
        SensorImporterHook ROS2ImuSensor();

        //! Retrieve a sensor importer hook which is used to map SDFormat sensor of type lidar into O3DE ROS2LidarSensorComponent
        SensorImporterHook ROS2LidarSensor();
    } // namespace ROS2SensorHooks
} // namespace ROS2::SDFormat
