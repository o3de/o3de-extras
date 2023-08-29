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
#include <ROS2/Communication/FlexiblePublisher.h>
#include <ROS2/Sensor/ROS2SensorComponent.h>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include "GNSSSensorConfiguration.h"

namespace ROS2
{
    //! Global Navigation Satellite Systems (GNSS) sensor component class
    //! It provides NavSatFix data of sensor's position in GNSS frame which is defined by GNSS origin offset
    //! Offset is provided as latitude [deg], longitude [deg], altitude [m] of o3de global frame
    //! It is assumed that o3de global frame overlaps with ENU coordinate system
    class ROS2GNSSSensorComponent : public ROS2SensorComponent
    {
    public:
        AZ_COMPONENT(ROS2GNSSSensorComponent, "{55B4A299-7FA3-496A-88F0-764C75B0E9A7}", ROS2SensorComponent);
        ROS2GNSSSensorComponent();
        ROS2GNSSSensorComponent(const SensorConfiguration& sensorConfiguration, const GNSSSensorConfiguration& gnssConfiguration);
        ~ROS2GNSSSensorComponent() = default;
        static void Reflect(AZ::ReflectContext* context);
        //////////////////////////////////////////////////////////////////////////
        // Component overrides
        void Activate() override;
        void Deactivate() override;
        //////////////////////////////////////////////////////////////////////////

    private:
        GNSSSensorConfiguration m_gnssConfiguration;

        //////////////////////////////////////////////////////////////////////////
        // ROS2SensorComponent overrides
        void FrequencyTick() override;
        //////////////////////////////////////////////////////////////////////////

        AZ::Transform GetCurrentPose() const;

        std::shared_ptr<FlexiblePublisher<sensor_msgs::msg::NavSatFix>> m_gnssPublisher;
        sensor_msgs::msg::NavSatFix m_gnssMsg;
    };

} // namespace ROS2
