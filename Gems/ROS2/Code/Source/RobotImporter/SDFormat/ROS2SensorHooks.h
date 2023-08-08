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
        SensorImporterHook ROS2CameraSensor();
        SensorImporterHook ROS2GNSSSensor();
        SensorImporterHook ROS2ImuSensor();
        SensorImporterHook ROS2LidarSensor();
    } // namespace ROS2SensorHooks
} // namespace ROS2::SDFormat
