/*
* Copyright (c) Contributors to the Open 3D Engine Project.
* For complete copyright and license terms please see the LICENSE at the root of this distribution.
*
* SPDX-License-Identifier: Apache-2.0 OR MIT
*
*/

#pragma once

#include "Sensor/SensorConfiguration.h"
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>

namespace ROS2
{
    void SensorConfiguration::Reflect(AZ::ReflectContext* context)
    {
        PublisherConfiguration::Reflect(context);
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<SensorConfiguration>()
                ->Version(1)
                ->Field("Visualise", &SensorConfiguration::m_visualise)
                ->Field("Publishing Enabled", &SensorConfiguration::m_publishingEnabled)
                ->Field("Frequency (HZ)", &SensorConfiguration::m_frequency)
                ->Field("Publishers", &SensorConfiguration::m_publishersConfigurations)
                ;

            if (AZ::EditContext* ec = serializeContext->GetEditContext())
            {
                ec->Class<SensorConfiguration>("ROS2 Sensor Component", "[Base component for sensors]")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &SensorConfiguration::m_visualise, "Visualise", "Visualise")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &SensorConfiguration::m_publishingEnabled, "Publishing Enabled", "Toggle publishing for topic")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &SensorConfiguration::m_frequency, "Frequency", "Frequency of publishing")
                        ->Attribute(AZ::Edit::Attributes::Min, 1)
                        ->Attribute(AZ::Edit::Attributes::Max, 100)
                        ->Attribute(AZ::Edit::Attributes::Step, 1)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &SensorConfiguration::m_publishersConfigurations, "Publishers", "Publishers")
                        ->Attribute(AZ::Edit::Attributes::ContainerCanBeModified, false)
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                        ->Attribute(AZ::Edit::Attributes::IndexedChildNameLabelOverride, &SensorConfiguration::GetPublisherLabel)
                    ;
            }
        }
    }

    AZStd::string SensorConfiguration::GetPublisherLabel(int index) const
    {
        return m_publishersConfigurations[index].m_type;
    }
}  // namespace ROS2
