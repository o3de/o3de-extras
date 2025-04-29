/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Math/Quaternion.h>
#include <AzCore/Math/Transform.h>
#include <AzCore/Math/Vector3.h>
#include <ROS2/Sensor/Events/PhysicsBasedSource.h>
#include <ROS2Sensors/Sensor/ROS2SensorComponentBase.h>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/publisher.hpp>

namespace ROS2Sensors
{
    //! Odometry sensor Component.
    //! It constructs and publishes an odometry message, which contains information about vehicle velocity and position in space.
    //! This is a ground truth "sensor", which can be helpful for development and machine learning.
    //! @see <a href="https://index.ros.org/p/nav_msgs/"> nav_msgs package. </a>
    class ROS2OdometrySensorComponent : public ROS2SensorComponentBase<ROS2::PhysicsBasedSource>
    {
    public:
        AZ_COMPONENT(ROS2OdometrySensorComponent, ROS2Sensors::ROS2OdometrySensorComponent, SensorBaseType);
        ROS2OdometrySensorComponent();
        ~ROS2OdometrySensorComponent() = default;
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void Reflect(AZ::ReflectContext* context);
        //////////////////////////////////////////////////////////////////////////
        // Component overrides
        void Activate() override;
        void Deactivate() override;
        //////////////////////////////////////////////////////////////////////////

    private:
        AzPhysics::SimulatedBodyHandle m_bodyHandle = AzPhysics::InvalidSimulatedBodyHandle;
        std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> m_odometryPublisher;
        nav_msgs::msg::Odometry m_odometryMsg;
        AZ::Transform m_initialTransform;

        void OnOdometryEvent(AzPhysics::SceneHandle sceneHandle);
    };
} // namespace ROS2Sensors
