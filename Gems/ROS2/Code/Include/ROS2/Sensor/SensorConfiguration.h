/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/RTTI/RTTI.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/std/containers/map.h>
#include <AzCore/std/string/string.h>
#include <ROS2/Communication/TopicConfiguration.h>
#include <ROS2/ROS2TypeIds.h>

namespace ROS2
{
    //! General configuration for sensors.
    //! All sensors can be set to a certain frequency, have their data published or not,
    //! and visualized or not.
    struct SensorConfiguration
    {
    public:
        AZ_TYPE_INFO(SensorConfiguration, SensorConfigurationTypeId);
        static void Reflect(AZ::ReflectContext* context)
        {
            if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
            {
                serializeContext->RegisterGenericType<AZStd::shared_ptr<ROS2::TopicConfiguration>>();
                serializeContext->RegisterGenericType<AZStd::map<AZStd::string, AZStd::shared_ptr<ROS2::TopicConfiguration>>>();
                serializeContext->Class<SensorConfiguration>()
                    ->Version(2)
                    ->Field("Visualize", &SensorConfiguration::m_visualize)
                    ->Field("Publishing Enabled", &SensorConfiguration::m_publishingEnabled)
                    ->Field("Frequency (HZ)", &SensorConfiguration::m_frequency)
                    ->Field("Publishers", &SensorConfiguration::m_publishersConfigurations);

                if (AZ::EditContext* ec = serializeContext->GetEditContext())
                {
                    ec->Class<SensorConfiguration>("ROS2 Sensor Configuration", "Sensor configuration")
                        ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->DataElement(AZ::Edit::UIHandlers::Default, &SensorConfiguration::m_visualize, "Visualize", "Visualize")
                        ->DataElement(
                            AZ::Edit::UIHandlers::Default,
                            &SensorConfiguration::m_publishingEnabled,
                            "Publishing Enabled",
                            "Toggle publishing for topic")
                        ->DataElement(
                            AZ::Edit::UIHandlers::Default, &SensorConfiguration::m_frequency, "Frequency", "Frequency of publishing [Hz]")
                        ->Attribute(AZ::Edit::Attributes::Min, SensorConfiguration::m_minFrequency)
                        ->DataElement(
                            AZ::Edit::UIHandlers::Default, &SensorConfiguration::m_publishersConfigurations, "Publishers", "Publishers")
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                        ->Attribute(AZ::Edit::Attributes::Visibility, AZ::Edit::PropertyVisibility::ShowChildrenOnly)
                        ->Attribute(AZ::Edit::Attributes::ContainerCanBeModified, false)
                        ->ElementAttribute(AZ::Edit::Attributes::AutoExpand, true)
                        ->ElementAttribute(AZ::Edit::Attributes::NameLabelOverride, "Publisher configuration");
                }
            }
        }

        //! ROS2 Publishers of this sensor.
        //! Some sensors can have more than one publisher (example: Camera).
        //! @note This map will typically hold 1-3 elements.
        AZStd::map<AZStd::string, ROS2::TopicConfiguration> m_publishersConfigurations;

        //! Frequency in Hz (1/s).
        //! Applies both to data acquisition and publishing.
        float m_frequency = 10.f;

        bool m_publishingEnabled = true; //!< Determines whether the sensor is publishing (sending data to ROS 2 ecosystem).
        bool m_visualize = true; //!< Determines whether the sensor is visualized in O3DE (for example, point cloud is drawn for LIDAR).
    private:
        // Frequency limit is once per day.
        static constexpr float m_minFrequency = AZStd::numeric_limits<float>::epsilon();
    };
} // namespace ROS2
