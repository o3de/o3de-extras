/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Math/Transform.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <ROS2/Sensor/Events/TickBasedSource.h>
#include <ROS2/Sensor/ROS2SensorComponentBase.h>
#include <ROS2Sensors/ROS2SensorsTypeIds.h>
#include <rclcpp/publisher.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <gps_msgs/msg/gps_fix.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <ROS2Sensors/GNSS/GNSSConfigurationRequestBus.h>
#include <Georeferencing/GeoreferenceStructures.h>
#include <RigidBody.h>

namespace ROS2Sensors
{
    //! Global Navigation Satellite Systems (GNSS) sensor component class
    //! It provides NavSatFix data of sensor's position in GNSS frame which is defined by GNSS origin offset
    //! Offset is provided as latitude [deg], longitude [deg], altitude [m] of o3de global frame
    //! It is assumed that o3de global frame overlaps with ENU coordinate system
    class ROS2GNSSSensorComponent : public ROS2::ROS2SensorComponentBase<ROS2::TickBasedSource>
                                  , protected GnssConfigurationRequestBus::Handler
    {
    public:
        using SensorBaseType = ROS2::ROS2SensorComponentBase<ROS2::TickBasedSource>;

        AZ_COMPONENT(ROS2GNSSSensorComponent, ROS2GNSSSensorComponentTypeId, SensorBaseType);
        ROS2GNSSSensorComponent();
        ROS2GNSSSensorComponent(const ROS2::SensorConfiguration& sensorConfiguration, const GnssSensorConfiguration& gnssConfiguration);
        ~ROS2GNSSSensorComponent() = default;
        static void Reflect(AZ::ReflectContext* context);

        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        //////////////////////////////////////////////////////////////////////////
        // Component overrides
        void Activate() override;
        void Deactivate() override;
        //////////////////////////////////////////////////////////////////////////

    private:
        // GnssConfigurationRequestBus::Handler overrides ..
        void SetConfiguration(const GnssSensorConfiguration& configuration) override;
        const GnssSensorConfiguration GetConfiguration() override;
        bool GetUseNavSat() override;
        void SetUseNavSat(bool useNavSat) override;
        bool GetUseGpsFix() override;
        void SetUseGpsFix(bool useGpsFix) override;
        bool GetUseQuaternion() override;
        void SetUseQuaternion(bool useQuaternion) override;

        ///! Requests gnss message publication.
        void FrequencyTick();

        ///! Publish as sensor_msgs::msg::NavSatFix
        void PublishNavSat(const Georeferencing::WGS::WGS84Coordinate&);

        ///! Publish as gps_msgs::msg::GPSFix
        void PublishGpsFix(const Georeferencing::WGS::WGS84Coordinate&, AzPhysics::RigidBody*);

         ///! Publish as geometry_msgs::msg::QuaternionStamped
        void PublishOrientation(AzPhysics::RigidBody*);

        GnssSensorConfiguration m_gnssConfiguration;
        AZ::Quaternion m_levelRotation;

        ///! Convenience template class for publisher
        template<typename TMsg>
        struct ROS2GNSSSensorPublisher
        {
            bool m_enabledPublish = false;
            std::shared_ptr<rclcpp::Publisher<TMsg>> m_publisher;
            TMsg m_msg;

            void Deactivate() {
                if(m_enabledPublish)
                {
                    m_publisher.reset();
                }
                m_enabledPublish = false;
            };
        };
        template <typename T> void ActivatePublisher(ROS2GNSSSensorPublisher<T>& publisher, bool enable, const ROS2::TopicConfiguration&);

        ROS2GNSSSensorPublisher<gps_msgs::msg::GPSFix> m_gpsfix_publisher;
        ROS2GNSSSensorPublisher<sensor_msgs::msg::NavSatFix> m_navsat_publisher;
        ROS2GNSSSensorPublisher<geometry_msgs::msg::QuaternionStamped> m_orientation_publisher;
    };

} // namespace ROS2Sensors
