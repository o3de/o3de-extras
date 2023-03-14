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
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/publisher.hpp>
#include <AzFramework/Physics/SimulatedBodies/RigidBody.h>

namespace ROS2
{
    //! Odometry sensor Component.
    //! It constructs and publishes an odometry message, which contains information about vehicle velocity and position in space.
    //! This is a ground truth "sensor", which can be helpful for development and machine learning.
    //! @see <a href="https://index.ros.org/p/nav_msgs/"> nav_msgs package. </a>
    class ROS2OdometrySensorComponent : public ROS2SensorComponent
    {
    public:
        AZ_COMPONENT(ROS2OdometrySensorComponent, "{61387448-63AA-4563-AF87-60C72B05B863}", ROS2SensorComponent);
        ROS2OdometrySensorComponent();
        ~ROS2OdometrySensorComponent() = default;
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
        AZ::Vector3 m_robotPose {0};
        AZ::Quaternion m_robotRotation {0,0,0,1};
        AzPhysics::SceneEvents::OnSceneActiveSimulatedBodiesEvent::Handler m_onSceneActiveSimulatedBodiesEvent;
        AzPhysics::RigidBody* m_rigidBodyPtr{ nullptr };
        AZ::Transform m_initialTransform;
    };
} // namespace ROS2
