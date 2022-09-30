/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "GNSS/ROS2GNSSSensorComponent.h"
#include "AzCore/Math/Matrix4x4.h"
#include "Frame/ROS2FrameComponent.h"
#include "ROS2/ROS2Bus.h"
#include "ROS2GemUtilities.h"
#include "Utilities/ROS2Conversions.h"
#include "Utilities/ROS2Names.h"

#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>

#include "GNSSFormatConversions.h"

namespace ROS2
{
    namespace Internal
    {
        const char* kGNSSMsgType = "sensor_msgs::msg::NavSatFix";
    }

    void ROS2GNSSSensorComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<ROS2GNSSSensorComponent, ROS2SensorComponent>()
                ->Version(1)
                ->Field("gnssOriginLatitude", &ROS2GNSSSensorComponent::m_gnssOriginLatitudeDeg)
                ->Field("gnssOriginLongitude", &ROS2GNSSSensorComponent::m_gnssOriginLongitudeDeg)
                ->Field("gnssOriginAltitude", &ROS2GNSSSensorComponent::m_gnssOriginAltitude);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<ROS2GNSSSensorComponent>("ROS2 GNSS Sensor", "GNSS sensor component")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ROS2GNSSSensorComponent::m_gnssOriginLatitudeDeg,
                        "Latitude offset",
                        "GNSS latitude position offset in degrees")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ROS2GNSSSensorComponent::m_gnssOriginLongitudeDeg,
                        "Longitude offset",
                        "GNSS longitude position offset in degrees")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ROS2GNSSSensorComponent::m_gnssOriginAltitude,
                        "Altitude offset",
                        "GNSS altitude position offset in meters");
            }
        }
    }

    ROS2GNSSSensorComponent::ROS2GNSSSensorComponent()
    {
        PublisherConfiguration pc;
        pc.m_type = Internal::kGNSSMsgType;
        pc.m_topic = "gnss";
        m_sensorConfiguration.m_frequency = 10;
        m_sensorConfiguration.m_publishersConfigurations.insert(AZStd::make_pair(Internal::kGNSSMsgType, pc));
    }

    void ROS2GNSSSensorComponent::Activate()
    {
        ROS2SensorComponent::Activate();
        auto ros2Node = ROS2Interface::Get()->GetNode();
        AZ_Assert(m_sensorConfiguration.m_publishersConfigurations.size() == 1, "Invalid configuration of publishers for GNSS sensor");

        const auto publisherConfig = m_sensorConfiguration.m_publishersConfigurations[Internal::kGNSSMsgType];
        const auto fullTopic = ROS2Names::GetNamespacedName(GetNamespace(), publisherConfig.m_topic);
        m_gnssPublisher = ros2Node->create_publisher<sensor_msgs::msg::NavSatFix>(fullTopic.data(), publisherConfig.GetQoS());

        m_gnssMsg.header.frame_id = "gnss_frame_id";
    }

    void ROS2GNSSSensorComponent::Deactivate()
    {
        ROS2SensorComponent::Deactivate();
        m_gnssPublisher.reset();
    }

    void ROS2GNSSSensorComponent::FrequencyTick()
    {
        const AZ::Vector3 currentPosition = GetCurrentPose().GetTranslation();
        const AZ::Vector3 currentPositionECEF =
            GNSS::ENUToECEF({ m_gnssOriginLatitudeDeg, m_gnssOriginLongitudeDeg, m_gnssOriginAltitude }, currentPosition);
        const AZ::Vector3 currentPositionWGS84 = GNSS::ECEFToWGS84(currentPositionECEF);

        m_gnssMsg.latitude = currentPositionWGS84.GetX();
        m_gnssMsg.longitude = currentPositionWGS84.GetY();
        m_gnssMsg.altitude = currentPositionWGS84.GetZ();

        m_gnssMsg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX;
        m_gnssMsg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GALILEO;

        m_gnssPublisher->publish(m_gnssMsg);
    }

    AZ::Transform ROS2GNSSSensorComponent::GetCurrentPose() const
    {
        auto* ros2Frame = Utils::GetGameOrEditorComponent<ROS2FrameComponent>(GetEntity());
        return ros2Frame->GetFrameTransform();
    }
} // namespace ROS2
