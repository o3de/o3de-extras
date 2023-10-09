/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Math/Transform.h>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/publisher.hpp>
#include <ROS2/Sensor/Events/PhysicsBasedSource.h>
#include <ROS2/Sensor/ROS2SensorComponentBase.h>

#include "ROS2OdometryCovariance.h"

namespace ROS2
{
    //! Wheel odometry sensor component.
    //! It constructs and publishes an odometry message, which contains information about the vehicle's velocity and position in space.
    //! This is a physical sensor that takes a vehicle's configuration and computes updates from the wheels' rotations.
    //! @see <a href="https://index.ros.org/p/nav_msgs/">nav_msgs package</a>.
    class ROS2WheelOdometryComponent
        : public ROS2SensorComponentBase<PhysicsBasedSource>
    {
    public:
        AZ_COMPONENT(ROS2WheelOdometryComponent, "{9bdb8c23-ac76-4c25-8d35-37aaa9f43fac}", SensorBaseType);
        ROS2WheelOdometryComponent();
        ~ROS2WheelOdometryComponent() = default;
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void Reflect(AZ::ReflectContext* context);
        //////////////////////////////////////////////////////////////////////////
        // Component overrides
        void Activate() override;
        void Deactivate() override;
        //////////////////////////////////////////////////////////////////////////

    private:
        std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> m_odometryPublisher;
        nav_msgs::msg::Odometry m_odometryMsg;
        AZ::Vector3 m_robotPose{ 0 };
        AZ::Quaternion m_robotRotation{ 0, 0, 0, 1 };
        ROS2OdometryCovariance m_poseCovariance;
        ROS2OdometryCovariance m_twistCovariance;

        void OnOdometryEvent();
        void OnPhysicsEvent(float physicsDeltaTime);
    };
} // namespace ROS2
