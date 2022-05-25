/*
* Copyright (c) Contributors to the Open 3D Engine Project.
* For complete copyright and license terms please see the LICENSE at the root of this distribution.
*
* SPDX-License-Identifier: Apache-2.0 OR MIT
*
*/
#pragma once

#include "PublisherConfiguration.h"
#include <AzCore/RTTI/RTTI.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/std/containers/map.h>
#include <AzCore/std/string/string.h>

namespace ROS2
{
    //! General configuration for sensors.
    //! All sensors can be set to a certain frequency, have their data published or not,
    //! and visualised or not.
    struct SensorConfiguration
    {
    public:
        AZ_TYPE_INFO(SensorConfiguration, "{4755363D-0B5A-42D7-BBEF-152D87BA10D7}");
        static void Reflect(AZ::ReflectContext* context);

        //! ROS2 Publishers of this sensor.
        //! Some sensors can have more than one publisher (example: Camera).
        //! This map will typically hold 1-3 elements.
        AZStd::map<AZStd::string, PublisherConfiguration> m_publishersConfigurations;

        //! Frequency in Hz (1/s).
        //! Applies both to data acquisition and publishing.
        // TODO - consider moving frequency, publishingEnabled to publisherConfiguration if any sensor has
        // a couple of publishers for which we want different values of these fields
        float m_frequency = 10;

        bool m_publishingEnabled = true;
        bool m_visualise = true;
    };
}  // namespace ROS2
