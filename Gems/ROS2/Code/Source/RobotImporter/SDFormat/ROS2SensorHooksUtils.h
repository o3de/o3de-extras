/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/std/string/string.h>
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
    } // namespace ROS2SensorHooks
} // namespace ROS2::SDFormat
