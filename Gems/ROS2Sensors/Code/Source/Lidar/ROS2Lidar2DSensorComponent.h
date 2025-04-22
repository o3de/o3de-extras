/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <Atom/RPI.Public/AuxGeom/AuxGeomDraw.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <ROS2/Sensor/Events/TickBasedSource.h>
#include <ROS2Sensors/Lidar/LidarRegistrarBus.h>
#include <ROS2Sensors/Lidar/LidarSystemBus.h>
#include <ROS2Sensors/Sensor/ROS2SensorComponentBase.h>
#include <rclcpp/publisher.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "LidarCore.h"
#include "LidarRaycaster.h"

namespace ROS2
{
    //! Lidar 2D sensor Component.
    //! Lidars (Light Detection and Ranging) emit laser light and measure it after reflection.
    //! Lidar Component allows customization of lidar type and behavior and encapsulates both simulation
    //! and data publishing. It requires ROS2FrameComponent.
    class ROS2Lidar2DSensorComponent : public ROS2SensorComponentBase<TickBasedSource, LidarSensorConfiguration>
    {
    public:
        AZ_COMPONENT(ROS2Lidar2DSensorComponent, ROS2Sensors::ROS2Lidar2DSensorComponentTypeId, SensorBaseType);
        ROS2Lidar2DSensorComponent();
        ROS2Lidar2DSensorComponent(const SensorConfiguration& sensorConfiguration, const LidarSensorConfiguration& lidarConfiguration);
        ~ROS2Lidar2DSensorComponent() = default;
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void Reflect(AZ::ReflectContext* context);
        //////////////////////////////////////////////////////////////////////////
        // Component overrides
        void Activate() override;
        void Deactivate() override;
        //////////////////////////////////////////////////////////////////////////

    private:
        //////////////////////////////////////////////////////////////////////////
        void FrequencyTick();

        void PublishRaycastResults(const RaycastResults& results);

        std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::LaserScan>> m_laserScanPublisher;

        LidarCore m_lidarCore;
    };
} // namespace ROS2
