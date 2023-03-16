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
#include <ROS2/Sensor/ROS2SensorComponent.h>
#include <rclcpp/publisher.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace ROS2
{
    //! An IMU (Inertial Measurement Unit) sensor Component.
    //! IMUs typically include gyroscopes, accelerometers and magnetometers. This component encapsulates data
    //! acquisition and its publishing to ROS2 ecosystem. IMU Component requires ROS2FrameComponent.
    class ROS2OldImuSensorComponent : public ROS2SensorComponent
    {
    public:
        AZ_COMPONENT(ROS2OldImuSensorComponent, "{6E0D7AC5-2556-4882-BB8B-7B442C5A470A}", ROS2SensorComponent);
        ROS2OldImuSensorComponent();
        ~ROS2OldImuSensorComponent() = default;
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
        //////////////////////////////////////////////////////////////////////////

        void InitializeImuMessage();
        double GetCurrentTimeInSec() const;
        AZ::Transform GetCurrentPose() const;

        std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Imu>> m_imuPublisher;

        sensor_msgs::msg::Imu m_imuMsg;
        double m_previousTime = 0.0;
        AZ::Transform m_previousPose = AZ::Transform::CreateIdentity();
        AZ::Vector3 m_previousLinearVelocity = AZ::Vector3::CreateZero();
    };
} // namespace ROS2
