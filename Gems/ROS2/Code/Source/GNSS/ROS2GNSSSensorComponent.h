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
#include <rclcpp/publisher.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

namespace ROS2
{
    //! Global Navigation Satellite Systems (GNSS) sensor component class
    //! It provides NavSatFix data of sensor's position in GNSS frame which is defined by GNSS origin offset
    //! Offset is provided as latitude [deg], longitude [deg], altitude [m] of o3de global frame
    //! It is assumed that o3de global frame overlaps with ENU coordinate system
    class ROS2GNSSSensorComponent : public ROS2SensorComponentBase<TickBasedSource>
    {
    public:
        AZ_COMPONENT(ROS2GNSSSensorComponent, "{55B4A299-7FA3-496A-88F0-764C75B0E9A7}", SensorBaseType);
        ROS2GNSSSensorComponent();
        ROS2GNSSSensorComponent(const SensorConfiguration& sensorConfiguration);
        ~ROS2GNSSSensorComponent() = default;
        static void Reflect(AZ::ReflectContext* context);

        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        //////////////////////////////////////////////////////////////////////////
        // Component overrides
        void Activate() override;
        void Deactivate() override;
        //////////////////////////////////////////////////////////////////////////

    private:
        ///! Requests gnss message publication.
        void FrequencyTick();

        //! Returns current entity position.
        //! @return Current entity position.
        [[nodiscard]] AZ::Transform GetCurrentPose() const;

        std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::NavSatFix>> m_gnssPublisher;
        sensor_msgs::msg::NavSatFix m_gnssMsg;
    };

} // namespace ROS2
