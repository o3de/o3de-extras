/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Math/Vector3.h>
#include <AzCore/RTTI/RTTI.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <ROS2Sensors/ROS2SensorsTypeIds.h>

namespace ROS2Sensors
{
    //! Constants used in GNSS sensor configuration and ROS2 messages.
    namespace GnssSensorConstants
    {
        inline constexpr char NavSatMessageType[] = "sensor_msgs::msg::NavSatFix";
        // keep original config name for backwards compatibility with existing prefabs
        inline constexpr char NavSatConfigName[] = "sensor_msgs::msg::NavSatFix";
        inline constexpr char GpsFixMessageType[] = "gps_msgs::msg::GPSFix";
        inline constexpr char GpsFixConfigName[] = "publish GPSFix";
        inline constexpr char QuaternionMessageType[] = "geometry_msgs::msg::QuaternionStamped";
        inline constexpr char QuaternionConfigName[] = "publish Orientation";
    } // namespace GnssSensorConstants

    //! A structure capturing configuration of an GNSS sensor.
    struct GnssSensorConfiguration
    {
        AZ_TYPE_INFO(GnssSensorConfiguration, GnssSensorConfigurationTypeId);
        static void Reflect(AZ::ReflectContext* context);

        bool m_useGpsFix = false; //!< use GNSS with orientation?
        bool m_useNavSat = true; //!< use "normal" GNSS?
        bool m_useQuaternion = false; //!< use orientation retrieved from GNSS?
    };
} // namespace ROS2Sensors
