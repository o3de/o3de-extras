/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <Atom/RPI.Public/AuxGeom/AuxGeomDraw.h>
#include <AzCore/Math/Vector3.h>
#include <ROS2/Sensor/ROS2SensorComponent.h>
#include <rclcpp/publisher.hpp>
#include <std_msgs/msg/bool.hpp>

namespace ROS2
{
    class ROS2ProximitySensor : public ROS2SensorComponent
    {
    public:
        AZ_COMPONENT(ROS2ProximitySensor, "{1f7b51f6-9450-4da4-9636-672a056e8812}", ROS2SensorComponent);
        ROS2ProximitySensor();
        ~ROS2ProximitySensor() = default;

        static void Reflect(AZ::ReflectContext* context);
        //////////////////////////////////////////////////////////////////////////
        // Component overrides
        void Activate() override;
        void Deactivate() override;
        //////////////////////////////////////////////////////////////////////////

    private:
        //////////////////////////////////////////////////////////////////////////
        // ROS2SensorComponent overrides
        void FrequencyTick() override;
        void Visualise() override;
        //////////////////////////////////////////////////////////////////////////

        AZ::Vector3 m_detectionDirection;
        float m_detectionDistance;
        bool m_wasObjectDetected = false;

        std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Bool>> m_detectionPublisher;

        AZ::RPI::AuxGeomDrawPtr m_drawQueue;
    };
} // namespace ROS2
