/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ROS2GNSSSensorComponent.h"
#include <AzCore/Math/Matrix4x4.h>
#include <ROS2/ROS2NamesBus.h>
#include <Source/RigidBodyComponent.h>
#include <Georeferencing/GeoreferenceBus.h>
#include <ROS2Sensors/GNSS/GNSSPostProcessingRequestBus.h>
#include <ROS2/Clock/ROS2ClockRequestBus.h>

namespace ROS2Sensors
{
    void ROS2GNSSSensorComponent::Reflect(AZ::ReflectContext* context)
    {
        GnssSensorConfiguration::Reflect(context);

        if (auto* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<ROS2GNSSSensorComponent, SensorBaseType>()->Version(5)->Field(
                "GnssSensorConfiguration",  &ROS2GNSSSensorComponent::m_gnssConfiguration);

            if (auto* editContext = serialize->GetEditContext())
            {
                editContext->Class<ROS2GNSSSensorComponent>("ROS2 GNSS Sensor", "GNSS sensor component")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->Attribute(AZ::Edit::Attributes::Icon, "Editor/Icons/Components/ROS2GNSSSensor.svg")
                    ->Attribute(AZ::Edit::Attributes::ViewportIcon, "Editor/Icons/Components/Viewport/ROS2GNSSSensor.svg")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ROS2GNSSSensorComponent::m_gnssConfiguration,
                        "GNSS sensor configuration",
                        "GNSS sensor configuration")
                    ->Attribute(AZ::Edit::Attributes::Visibility, AZ::Edit::PropertyVisibility::ShowChildrenOnly);
            }
        }
    }

    ROS2GNSSSensorComponent::ROS2GNSSSensorComponent()
    {
        ROS2::TopicConfiguration pc;
        pc.m_type = GnssSensorConstants::NavSatMessageType;
        pc.m_topic = "gnss";
        m_sensorConfiguration.m_frequency = 10;
        m_sensorConfiguration.m_publishersConfigurations.insert(AZStd::make_pair(GnssSensorConstants::NavSatConfigName, pc));

        ROS2::TopicConfiguration gpsfix;
        gpsfix.m_type = GnssSensorConstants::GpsFixMessageType;
        gpsfix.m_topic = "gnss_oriented";
        m_sensorConfiguration.m_publishersConfigurations.insert(AZStd::make_pair(GnssSensorConstants::GpsFixConfigName, gpsfix));

        ROS2::TopicConfiguration gpsQuaternion;
        gpsQuaternion.m_type = GnssSensorConstants::QuaternionMessageType;
        gpsQuaternion.m_topic = "gnss_orientation";
        m_sensorConfiguration.m_publishersConfigurations.insert(AZStd::make_pair(GnssSensorConstants::QuaternionConfigName, gpsQuaternion));

        m_levelRotation = AZ::Quaternion::CreateIdentity();
    }

    ROS2GNSSSensorComponent::ROS2GNSSSensorComponent(const ROS2::SensorConfiguration& sensorConfiguration, const GnssSensorConfiguration& gnssConfiguration)
       : m_gnssConfiguration(gnssConfiguration)
    {
        m_sensorConfiguration = sensorConfiguration;
    }

    void ROS2GNSSSensorComponent::Activate()
    {
        ROS2SensorComponentBase::Activate();
        AZ_Assert(m_sensorConfiguration.m_publishersConfigurations.size() == 1, "Invalid configuration of publishers for GNSS sensor");

        auto & publisherConfig = m_sensorConfiguration.m_publishersConfigurations;
        ActivatePublisher(m_gpsfix_publisher,
                          GetUseGpsFix(),
                          publisherConfig[GnssSensorConstants::GpsFixConfigName]);
        ActivatePublisher(m_navsat_publisher,
                          GetUseNavSat(),
                          publisherConfig[GnssSensorConstants::NavSatConfigName]);
        ActivatePublisher(m_orientation_publisher,
                          GetUseQuaternion(),
                          publisherConfig[GnssSensorConstants::QuaternionConfigName]);

        // get rotation of Level vs. ENU GNSS-Orientation
        Georeferencing::GeoreferenceRequestsBus::BroadcastResult(m_levelRotation, &Georeferencing::GeoreferenceRequests::GetRotationFromLevelToENU);

        const auto* ros2Frame = GetEntity()->FindComponent<ROS2::ROS2FrameComponent>();
        AZ_Assert(ros2Frame, "Invalid component pointer value");
        m_gpsfix_publisher.m_msg.header.frame_id =
            m_navsat_publisher.m_msg.header.frame_id =
                m_orientation_publisher.m_msg.header.frame_id =
                    ros2Frame->GetNamespacedFrameID().data();

        GnssConfigurationRequestBus::Handler::BusConnect(GetEntityId());
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

    template <typename T> void ROS2GNSSSensorComponent::ActivatePublisher(
                                                          ROS2GNSSSensorPublisher<T>& publisher,
                                                          bool enable,
                                                          const ROS2::TopicConfiguration& publisherConfig)
    {
        publisher.m_enabledPublish = enable;
        if(publisher.m_enabledPublish)
        {
            AZStd::string fullTopic;
            ROS2::ROS2NamesRequestBus::BroadcastResult(
            fullTopic, &ROS2::ROS2NamesRequestBus::Events::GetNamespacedName, GetNamespace(), publisherConfig.m_topic);
            auto ros2Node = ROS2::ROS2Interface::Get()->GetNode();
            publisher.m_publisher = ros2Node->create_publisher<T>(fullTopic.data(), publisherConfig.GetQoS());
        }
    }

    void ROS2GNSSSensorComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("ROS2GNSSSensor"));
    }

    void ROS2GNSSSensorComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        // do not require PhysicsDynamicRigidBodyService due to backwards compatibility
        required.push_back(AZ_CRC_CE("ROS2Frame"));
    }

    void ROS2GNSSSensorComponent::Deactivate()
    {
        GnssConfigurationRequestBus::Handler::BusDisconnect(GetEntityId());
        StopSensor();
        m_gpsfix_publisher.Deactivate();
        m_navsat_publisher.Deactivate();
        m_orientation_publisher.Deactivate();
        ROS2SensorComponentBase::Deactivate();
    }

    void ROS2GNSSSensorComponent::FrequencyTick()
    {
        using namespace Georeferencing;
        AZ::Vector3 currentPosition{ 0.0f };
        AZ::TransformBus::EventResult(currentPosition, GetEntityId(), &AZ::TransformBus::Events::GetWorldTranslation);

        WGS::WGS84Coordinate currentPositionWGS84;
        GeoreferenceRequestsBus::BroadcastResult(currentPositionWGS84, &GeoreferenceRequests::ConvertFromLevelToWGS84, currentPosition);

        AzPhysics::RigidBody* rigidBody = nullptr;
        if(m_gpsfix_publisher.m_enabledPublish || m_orientation_publisher.m_enabledPublish)
        {
            AZ::EntityId entityId = GetEntityId();
            // fetch current orientation relative to level
            Physics::RigidBodyRequestBus::EventResult(rigidBody,
                                                      entityId,
                                                      &Physics::RigidBodyRequests::GetRigidBody);
            if(!rigidBody)
            {
              // if ridid body missing
              // issue error message and stop sensor
              // unless also publishing sensor_msgs::msg::NavSatFix
              // In such a case issue warning only and disable orientation sensors
              if(m_navsat_publisher.m_enabledPublish)
              {
                AZ_Warning(
                    "ROS2GNSSSensorComponent",
                     false,
                    "Entity %s does not have a rigid body - stopping GNSS orientation sensors.",
                    entityId.ToString().c_str());
                m_gpsfix_publisher.Deactivate();
                m_orientation_publisher.Deactivate();
              }
              else
              {
                AZ_Error(
                    "ROS2GNSSSensorComponent",
                     false,
                    "Entity %s does not have a rigid body - stopping all GNSS sensors.",
                    entityId.ToString().c_str());
                StopSensor();
              }
            }
        }

        m_gpsfix_publisher.m_msg.header.stamp =
            m_orientation_publisher.m_msg.header.stamp =
                m_gpsfix_publisher.m_msg.header.stamp = ROS2::ROS2ClockInterface::Get()->GetROSTimestamp();
        PublishGpsFix(currentPositionWGS84, rigidBody);
        PublishNavSat(currentPositionWGS84);
        PublishOrientation(rigidBody);
    }

    void ROS2GNSSSensorComponent::PublishNavSat(const Georeferencing::WGS::WGS84Coordinate& currentPositionWGS84)
    {
        if(m_navsat_publisher.m_enabledPublish)
        {
            m_navsat_publisher.m_msg.latitude = currentPositionWGS84.m_latitude;
            m_navsat_publisher.m_msg.longitude = currentPositionWGS84.m_longitude;
            m_navsat_publisher.m_msg.altitude = currentPositionWGS84.m_altitude;

            m_navsat_publisher.m_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX;
            m_navsat_publisher.m_msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
            GNSSPostProcessingRequestBus::Event(GetEntityId(),
                                                &GNSSPostProcessingRequests::ApplyPostProcessing,
                                                m_navsat_publisher.m_msg);

            m_navsat_publisher.m_publisher->publish(m_navsat_publisher.m_msg);
        }
    }

    void ROS2GNSSSensorComponent::PublishGpsFix(const Georeferencing::WGS::WGS84Coordinate& currentPositionWGS84,
                                                AzPhysics::RigidBody* rigidBody)
    {
        if(m_gpsfix_publisher.m_enabledPublish)
        {
            m_gpsfix_publisher.m_msg.latitude = currentPositionWGS84.m_latitude;
            m_gpsfix_publisher.m_msg.longitude = currentPositionWGS84.m_longitude;
            m_gpsfix_publisher.m_msg.altitude = currentPositionWGS84.m_altitude;
            AZ::Vector3 eulerAngles(0.0);
            if(rigidBody)
            {
                const AZ::Quaternion orientationLevel = rigidBody->GetOrientation();
                // determine current orientation relative to GNSS ENU coordinates
                const AZ::Quaternion orientationENU = m_levelRotation * orientationLevel;
                eulerAngles = orientationENU.GetEulerDegreesXYZ();
            }
            // get deviation angle towards east
            float dip = eulerAngles.GetZ();
            // map angle according Sepentrio GNSS: 2. quadrant is [-270°..-180°]
            if(90.0 < dip)
            {
                dip -= 360.0;
            }
            m_gpsfix_publisher.m_msg.dip = dip;
            m_gpsfix_publisher.m_msg.status.status = gps_msgs::msg::GPSStatus::STATUS_SBAS_FIX;

            m_gpsfix_publisher.m_publisher->publish(m_gpsfix_publisher.m_msg);
        }
    }

    void ROS2GNSSSensorComponent::PublishOrientation(AzPhysics::RigidBody* rigidBody)
    {
        if(m_orientation_publisher.m_enabledPublish && rigidBody)
        {
            const AZ::Quaternion orientationLevel = rigidBody->GetOrientation();
            // determine current orientation relative to GNSS ENU coordinates
            const AZ::Quaternion orientationENU = m_levelRotation * orientationLevel;

            m_orientation_publisher.m_msg.quaternion.x = orientationENU.GetX();
            m_orientation_publisher.m_msg.quaternion.y = orientationENU.GetY();
            m_orientation_publisher.m_msg.quaternion.z = orientationENU.GetZ();
            m_orientation_publisher.m_msg.quaternion.w = orientationENU.GetW();

            m_orientation_publisher.m_publisher->publish(m_orientation_publisher.m_msg);
        }
    }

    void ROS2GNSSSensorComponent::SetConfiguration(const GnssSensorConfiguration& configuration)
    {
        m_gnssConfiguration = configuration;
    }

    const GnssSensorConfiguration ROS2GNSSSensorComponent::GetConfiguration()
    {
        return m_gnssConfiguration;
    }

    bool ROS2GNSSSensorComponent::GetUseNavSat()
    {
        return GetConfiguration().m_useNavSat;
    }

    void ROS2GNSSSensorComponent::SetUseNavSat(bool useNavSat)
    {
        auto gnssConfiguration = GetConfiguration();
        gnssConfiguration.m_useNavSat = useNavSat;
        SetConfiguration(gnssConfiguration);
    }

    bool ROS2GNSSSensorComponent::GetUseGpsFix()
    {
        return GetConfiguration().m_useGpsFix;
    }

    void ROS2GNSSSensorComponent::SetUseGpsFix(bool useGpsFix)
    {
        auto gnssConfiguration = GetConfiguration();
        gnssConfiguration.m_useGpsFix = useGpsFix;
        SetConfiguration(gnssConfiguration);
    }

    bool ROS2GNSSSensorComponent::GetUseQuaternion()
    {
        return GetConfiguration().m_useQuaternion;
    }

    void ROS2GNSSSensorComponent::SetUseQuaternion(bool useQuaternion)
    {
        auto gnssConfiguration = GetConfiguration();
        gnssConfiguration.m_useQuaternion = useQuaternion;
        SetConfiguration(gnssConfiguration);
    }
} // namespace ROS2Sensors
