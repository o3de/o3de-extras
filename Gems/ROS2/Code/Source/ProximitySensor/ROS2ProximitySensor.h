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
#include <ROS2/Utilities/ROS2ErrorHandler.h>
#include <rclcpp/publisher.hpp>
#include <std_msgs/msg/bool.hpp>

namespace ROS2
{
    //! Simple proximity sensor based on raycasting
    //! This component publishes a bool topic depending on the object presence
    class ROS2ProximitySensor
        : public ROS2SensorComponent
        , public ROS2ErrorHandler
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
        void Visualize() override;
        //////////////////////////////////////////////////////////////////////////

        AZ::Vector3 m_detectionDirection = AZ::Vector3::CreateAxisX();
        float m_detectionDistance = 0.f;
        std::optional<AZ::Vector3> m_position;

        std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Bool>> m_detectionPublisher;

        AZ::RPI::AuxGeomDrawPtr m_drawQueue;
    };
} // namespace ROS2
