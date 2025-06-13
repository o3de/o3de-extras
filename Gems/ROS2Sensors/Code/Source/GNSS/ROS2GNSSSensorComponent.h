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

namespace ROS2Sensors
{
    //! Global Navigation Satellite Systems (GNSS) sensor component class
    //! It provides NavSatFix data of sensor's position in GNSS frame which is defined by GNSS origin offset
    //! Offset is provided as latitude [deg], longitude [deg], altitude [m] of o3de global frame
    //! It is assumed that o3de global frame overlaps with ENU coordinate system
    class ROS2GNSSSensorComponent : public ROS2::ROS2SensorComponentBase<ROS2::TickBasedSource>
    {
    public:
        using SensorBaseType = ROS2::ROS2SensorComponentBase<ROS2::TickBasedSource>;

        AZ_COMPONENT(ROS2GNSSSensorComponent, ROS2GNSSSensorComponentTypeId, SensorBaseType);
        ROS2GNSSSensorComponent();
        ROS2GNSSSensorComponent(const ROS2::SensorConfiguration& sensorConfiguration);
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

} // namespace ROS2Sensors
