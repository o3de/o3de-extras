/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <rclcpp/publisher.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "Sensor/ROS2SensorComponent.h"

#include <AzCore/Component/Component.h>

#include "CameraSensor.h"

namespace ROS2
{
    //! ROS2 Camera sensor component class
    //! Allows turning an entity into a camera sensor
    //! Can be parametrized with following values:
    //!   - camera name
    //!   - camera image width and height in pixels
    //!   - camera vertical field of view in degrees
    //! Camera frustum is facing negative Z axis; image plane is parallel to X,Y plane: X - right, Y - up
    class ROS2CameraSensorComponent
        : public ROS2SensorComponent
    {
    public:
        ROS2CameraSensorComponent();
        ~ROS2CameraSensorComponent() override = default;
        AZ_COMPONENT(ROS2CameraSensorComponent, "{3C6B8AE6-9721-4639-B8F9-D8D28FD7A071}", ROS2SensorComponent);
        static void Reflect(AZ::ReflectContext* context);

        void Activate() override;
        void Deactivate() override;

    private:
        float m_VerticalFieldOfViewDeg = 90.0f;
        int m_width = 640;
        int m_height = 480;
        AZStd::string m_cameraName = "camera";

        void FrequencyTick() override;

        std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> m_imagePublisher;
        std::optional<CameraSensor> m_cameraSensor;
    };
}  // namespace ROS2
