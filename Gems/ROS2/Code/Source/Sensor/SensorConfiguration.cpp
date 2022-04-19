/*
* Copyright (c) Contributors to the Open 3D Engine Project.
* For complete copyright and license terms please see the LICENSE at the root of this distribution.
*
* SPDX-License-Identifier: Apache-2.0 OR MIT
*
*/

#pragma once

#include "Sensor/SensorConfiguration.h"

namespace ROS2
{
    void SensorConfiguration::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<SensorConfiguration>()
                ->Version(1)
                ->Field("Topic", &SensorConfiguration::m_topic)
                ->Field("Publishing Enabled", &SensorConfiguration::m_publishingEnabled)
                ->Field("Frequency (HZ)", &SensorConfiguration::m_frequency)
                ->Field("Visualise", &SensorConfiguration::m_visualise)
                ;

            if (AZ::EditContext* ec = serializeContext->GetEditContext())
            {
                ec->Class<SensorConfiguration>("ROS2 Sensor Component", "[Base component for sensors]")
                        ->DataElement(AZ::Edit::UIHandlers::Default, &SensorConfiguration::m_topic, "Topic", "Topic")
                        ->DataElement(AZ::Edit::UIHandlers::Default, &SensorConfiguration::m_publishingEnabled, "Publishing Enabled", "Publishing Enabled")
                        ->DataElement(AZ::Edit::UIHandlers::Default, &SensorConfiguration::m_frequency, "Frequency", "Frequency (HZ)")
                        ->DataElement(AZ::Edit::UIHandlers::Default, &SensorConfiguration::m_visualise, "Visualise", "Visualise")
                        ;
            }
        }
    }
}  // namespace ROS2

