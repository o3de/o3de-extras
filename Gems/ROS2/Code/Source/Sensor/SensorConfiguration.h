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
#include <AzCore/std/smart_ptr/shared_ptr.h>
#include <AzCore/std/containers/vector.h>
#include "PublisherConfiguration.h"

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

        // Will typically be 1-3 elements (3 max for known sensors).
        AZStd::vector<PublisherConfiguration> m_publishersConfigurations;

        // TODO - consider moving frequency, publishingEnabled to publisherConfiguration if any sensor has
        // a couple of publishers for which we want different values of these fields
        float m_frequency = 10;
        bool m_publishingEnabled = true;
        bool m_visualise = true;

    private:
        AZStd::string GetPublisherLabel(int index) const;
    };
}  // namespace ROS2

