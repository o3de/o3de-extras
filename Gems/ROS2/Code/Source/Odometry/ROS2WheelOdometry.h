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
#include <AzFramework/Physics/PhysicsSystem.h>
#include <AzFramework/Physics/Common/PhysicsEvents.h>

namespace ROS2
{

   class ROS2WheelOdometryComponent : public ROS2SensorComponent
   {
   public:
       AZ_COMPONENT(ROS2WheelOdometryComponent, "{9bdb8c23-ac76-4c25-8d35-37aaa9f43fac}", ROS2SensorComponent);
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
       AZ::Vector3 m_robotPose {0};
       AZ::Quaternion m_robotRotation {0,0,0,1};
       float m_gtWeightLinear {0.f};
       float m_gtWeightAngular {0.f};
       AzPhysics::RigidBody* m_rigidBodyPtr{ nullptr };
       AzPhysics::SceneEvents::OnSceneActiveSimulatedBodiesEvent::Handler m_onSceneActiveSimulatedBodiesEvent;
   };
} // namespace ROS2
