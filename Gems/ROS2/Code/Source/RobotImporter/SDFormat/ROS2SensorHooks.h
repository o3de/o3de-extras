/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <ROS2/RobotImporter/SDFormatSensorImporterHook.h>

namespace ROS2::SDFormat
{
    namespace ROS2SensorHooks
    {
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
