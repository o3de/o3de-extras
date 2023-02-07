/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>
#include <ROS2/Sensor/SensorConfiguration.h>

namespace ROS2
{
    void SensorConfiguration::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->RegisterGenericType<AZStd::shared_ptr<TopicConfiguration>>();
            serializeContext->RegisterGenericType<AZStd::map<AZStd::string, AZStd::shared_ptr<TopicConfiguration>>>();
            serializeContext->Class<SensorConfiguration>()
                ->Version(2)
                ->Field("Visualise", &SensorConfiguration::m_visualise)
                ->Field("Publishing Enabled", &SensorConfiguration::m_publishingEnabled)
                ->Field("Frequency (HZ)", &SensorConfiguration::m_frequency)
                ->Field("Publishers", &SensorConfiguration::m_publishersConfigurations);

            if (AZ::EditContext* ec = serializeContext->GetEditContext())
            {
                ec->Class<SensorConfiguration>("ROS2 Sensor Configuration", "Sensor configuration")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &SensorConfiguration::m_visualise, "Visualise", "Visualise")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &SensorConfiguration::m_publishingEnabled,
                        "Publishing Enabled",
                        "Toggle publishing for topic")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default, &SensorConfiguration::m_frequency, "Frequency", "Frequency of publishing [Hz]")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default, &SensorConfiguration::m_publishersConfigurations, "Publishers", "Publishers")
                    ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->Attribute(AZ::Edit::Attributes::Visibility, AZ::Edit::PropertyVisibility::ShowChildrenOnly)
                    ->Attribute(AZ::Edit::Attributes::ContainerCanBeModified, false)
                    ->ElementAttribute(AZ::Edit::Attributes::AutoExpand, true);
            }
        }
    }
} // namespace ROS2
