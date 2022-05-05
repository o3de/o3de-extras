/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <rclcpp/publisher.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <AzCore/Serialization/SerializeContext.h>
#include "Sensor/ROS2SensorComponent.h"
#include "Lidar/LidarTemplate.h"
#include "Lidar/LidarRaycaster.h"

namespace ROS2
{
    class ROS2LidarSensorComponent
        : public ROS2SensorComponent
    {
    public:
        AZ_COMPONENT(ROS2LidarSensorComponent, "{502A955F-7742-4E23-AD77-5E4063739DCA}", ROS2SensorComponent);
        ROS2LidarSensorComponent();
        ~ROS2LidarSensorComponent() = default;
        static void Reflect(AZ::ReflectContext* context);
        void Activate() override;
        void Deactivate() override;

    private:
        void FrequencyTick() override;

        LidarTemplate::LidarModel m_lidarModel = LidarTemplate::Generic3DLidar;
        LidarRaycaster m_lidarRaycaster;
        std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> m_pointCloudPublisher;
        // TODO - also add a data acquisition implementation choice (and consider propagating abstraction upwards)
    };
}  // namespace ROS2
