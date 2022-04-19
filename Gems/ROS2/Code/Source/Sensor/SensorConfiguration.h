/*
* Copyright (c) Contributors to the Open 3D Engine Project.
* For complete copyright and license terms please see the LICENSE at the root of this distribution.
*
* SPDX-License-Identifier: Apache-2.0 OR MIT
*
*/

#pragma once

#include <AzCore/RTTI/RTTI.h>
#include <AzCore/Serialization/SerializeContext.h>

namespace ROS2
{
    /// General configuration for sensors
    struct SensorConfiguration
    {
    public:
        AZ_RTTI(SensorConfiguration, "{4755363D-0B5A-42D7-BBEF-152D87BA10D7}");

        SensorConfiguration() = default;
        virtual ~SensorConfiguration() = default;

        static void Reflect(AZ::ReflectContext* context);

        // TODO - publishing-related data
        AZStd::string m_topic = "default_topic"; // TODO - apply namespace, default to standard names per type, validation
        bool m_publishingEnabled = true; // TODO - support this flag
        float m_frequency = 10;
        // TODO - add QoS here (struct, mapped to ros2 QoS).

        bool m_visualise = true;
    };
}  // namespace ROS2

