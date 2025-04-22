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

namespace ROS2
{
    //! This sensor does not have any configuration parameters.
    //! This structure is required to be able to use the SensorConfigurationRequestBus.
    struct ROS2OdometrySensorConfiguration
    {
        AZ_RTTI(ROS2OdometrySensorConfiguration, "{04c5d154-6fa7-4c12-a4e3-025ac853198a}");

        ROS2OdometrySensorConfiguration() = default;
        virtual ~ROS2OdometrySensorConfiguration() = default;

        static void Reflect(AZ::ReflectContext* context);
    };

    //! Odometry sensor Component.
    //! It constructs and publishes an odometry message, which contains information about vehicle velocity and position in space.
    //! This is a ground truth "sensor", which can be helpful for development and machine learning.
    //! @see <a href="https://index.ros.org/p/nav_msgs/"> nav_msgs package. </a>
    class ROS2OdometrySensorComponent : public ROS2SensorComponentBase<PhysicsBasedSource, ROS2OdometrySensorConfiguration>
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
        // ConfigurationBus overrides
        const ROS2OdometrySensorConfiguration GetConfiguration() const override;
        void SetConfiguration(const ROS2OdometrySensorConfiguration configuration) override;

        AzPhysics::SimulatedBodyHandle m_bodyHandle = AzPhysics::InvalidSimulatedBodyHandle;
        std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> m_odometryPublisher;
        nav_msgs::msg::Odometry m_odometryMsg;
        AZ::Transform m_initialTransform;

        void OnOdometryEvent(AzPhysics::SceneHandle sceneHandle);
    };
} // namespace ROS2
