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
#include <AzCore/Serialization/SerializeContext.h>
#include <AzFramework/Physics/Common/PhysicsEvents.h>
#include <AzFramework/Physics/PhysicsScene.h>
#include <AzFramework/Physics/PhysicsSystem.h>
#include <AzFramework/Physics/SimulatedBodies/RigidBody.h>
#include <ROS2/Communication/FlexiblePublisher.h>
#include <ROS2/Sensor/ROS2SensorComponent.h>
#include <ROS2/Utilities/PhysicsCallbackHandler.h>
#include <nav_msgs/msg/odometry.hpp>

namespace ROS2
{
    //! Odometry sensor Component.
    //! It constructs and publishes an odometry message, which contains information about vehicle velocity and position in space.
    //! This is a ground truth "sensor", which can be helpful for development and machine learning.
    //! @see <a href="https://index.ros.org/p/nav_msgs/"> nav_msgs package. </a>
    class ROS2OdometrySensorComponent
        : public ROS2SensorComponent
        , public ROS2::Utils::PhysicsCallbackHandler
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
        std::shared_ptr<FlexiblePublisher<nav_msgs::msg::Odometry>> m_odometryPublisher;
        nav_msgs::msg::Odometry m_odometryMsg;
        AZ::Transform m_initialTransform;

    private:
        // ROS2SensorComponent overrides ...
        void SetupRefreshLoop() override;

        // ROS2::Utils::PhysicsCallbackHandler overrides ...
        void OnPhysicsSimulationFinished(AzPhysics::SceneHandle sceneHandle, float deltaTime) override;
        void OnPhysicsInitialization(AzPhysics::SceneHandle sceneHandle) override;

        //! Handler to simulated physical body
        AzPhysics::SimulatedBodyHandle m_bodyHandle = AzPhysics::InvalidSimulatedBodyHandle;
    };
} // namespace ROS2
