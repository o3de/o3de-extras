/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ROS2GNSSSensorComponent.h"
#include <AzCore/Math/Matrix4x4.h>
#include <ROS2/Frame/ROS2FrameComponent.h>
#include <ROS2/ROS2GemUtilities.h>
#include <ROS2/Utilities/ROS2Names.h>

#include "Georeference/GNSSFormatConversions.h"
#include <ROS2/GNSS/GNSSPostProcessingRequestBus.h>
#include <ROS2/Georeference/GeoreferenceBus.h>

namespace ROS2
{
    namespace
    {
        const char* GNSSMsgType = "sensor_msgs::msg::NavSatFix";
    }

    void ROS2GNSSSensorComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<ROS2GNSSSensorComponent, SensorBaseType>()->Version(4);

            if (auto* editContext = serialize->GetEditContext())
            {
                editContext->Class<ROS2GNSSSensorComponent>("ROS2 GNSS Sensor", "GNSS sensor component")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->Attribute(AZ::Edit::Attributes::Icon, "Editor/Icons/Components/ROS2GNSSSensor.svg")
                    ->Attribute(AZ::Edit::Attributes::ViewportIcon, "Editor/Icons/Components/Viewport/ROS2GNSSSensor.svg")
                    ->Attribute(AZ::Edit::Attributes::Visibility, AZ::Edit::PropertyVisibility::ShowChildrenOnly);
            }
        }
    }

    ROS2GNSSSensorComponent::ROS2GNSSSensorComponent()
    {
        TopicConfiguration pc;
        pc.m_type = GNSSMsgType;
        pc.m_topic = "gnss";
        m_sensorConfiguration.m_frequency = 10;
        m_sensorConfiguration.m_publishersConfigurations.insert(AZStd::make_pair(GNSSMsgType, pc));
    }

    ROS2GNSSSensorComponent::ROS2GNSSSensorComponent(const SensorConfiguration& sensorConfiguration)
    {
        m_sensorConfiguration = sensorConfiguration;
    }

    void ROS2GNSSSensorComponent::Activate()
    {
        auto ros2Node = ROS2Interface::Get()->GetNode();
        AZ_Assert(m_sensorConfiguration.m_publishersConfigurations.size() == 1, "Invalid configuration of publishers for GNSS sensor");

        const auto publisherConfig = m_sensorConfiguration.m_publishersConfigurations[GNSSMsgType];
        const auto fullTopic = ROS2Names::GetNamespacedName(GetNamespace(), publisherConfig.m_topic);
        m_gnssPublisher = ros2Node->create_publisher<sensor_msgs::msg::NavSatFix>(fullTopic.data(), publisherConfig.GetQoS());

        m_gnssMsg.header.frame_id = "gnss_frame_id";

        StartSensor(
            m_sensorConfiguration.m_frequency,
            [this]([[maybe_unused]] auto&&... args)
            {
                if (!m_sensorConfiguration.m_publishingEnabled)
                {
                    return;
                }
                FrequencyTick();
            });
    }

    void ROS2GNSSSensorComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("ROS2GNSSSensor"));
    }

    void ROS2GNSSSensorComponent::Deactivate()
    {
        StopSensor();
        m_gnssPublisher.reset();
    }

    void ROS2GNSSSensorComponent::FrequencyTick()
    {
        AZ::Vector3 currentPosition{ 0.0f };
        AZ::TransformBus::EventResult(currentPosition, GetEntityId(), &AZ::TransformBus::Events::GetWorldTranslation);

        WGS::WGS84Coordinate currentPositionWGS84;
        ROS2::GeoreferenceRequestsBus::BroadcastResult(
            currentPositionWGS84, &GeoreferenceRequests::ConvertFromLevelToWSG84, currentPosition);

        m_gnssMsg.latitude = currentPositionWGS84.m_latitude;
        m_gnssMsg.longitude = currentPositionWGS84.m_longitude;
        m_gnssMsg.altitude = currentPositionWGS84.m_altitude;

        m_gnssMsg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX;
        m_gnssMsg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
        GNSSPostProcessingRequestBus::Event(GetEntityId(), &GNSSPostProcessingRequests::ApplyPostProcessing, m_gnssMsg);

        m_gnssPublisher->publish(m_gnssMsg);
    }

} // namespace ROS2
