/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Math/Transform.h>
#include <ROS2/Sensor/Events/PhysicsBasedSource.h>
#include <ROS2/Sensor/ROS2SensorComponentBase.h>
#include <ROS2Sensors/Odometry/ROS2OdometryCovariance.h>
#include <ROS2Sensors/Odometry/WheelOdometryConfigurationRequestBus.h>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/publisher.hpp>

namespace ROS2Sensors
{
    class JsonROS2WheelOdometryComponentConfigSerializer : public AZ::BaseJsonSerializer
    {
    public:
        AZ_RTTI(JsonROS2WheelOdometryComponentConfigSerializer, "{3f7f9be6-d964-4a55-b856-c03cc5754df0}", AZ::BaseJsonSerializer);
        AZ_CLASS_ALLOCATOR_DECL;

        AZ::JsonSerializationResult::Result Load(
            void* outputValue,
            const AZ::Uuid& outputValueTypeId,
            const rapidjson::Value& inputValue,
            AZ::JsonDeserializerContext& context) override;
    };

    //! Wheel odometry sensor component.
    //! It constructs and publishes an odometry message, which contains information about the vehicle's velocity and position in space.
    //! This is a physical sensor that takes a vehicle's configuration and computes updates from the wheels' rotations.
    //! @see <a href="https://index.ros.org/p/nav_msgs/">nav_msgs package</a>.
    class ROS2WheelOdometryComponent
        : public ROS2::ROS2SensorComponentBase<ROS2::PhysicsBasedSource>
        , protected WheelOdometryConfigurationRequestBus::Handler
    {
        friend class JsonROS2WheelOdometryComponentConfigSerializer;

    public:
        using SensorBaseType = ROS2::ROS2SensorComponentBase<ROS2::PhysicsBasedSource>;

        AZ_COMPONENT(ROS2WheelOdometryComponent, ROS2WheelOdometryComponentTypeId, SensorBaseType);
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
        //////////////////////////////////////////////////////////////////////////
        // WheelOdometryConfigurationRequestBus::Handler overrides
        const WheelOdometrySensorConfiguration GetConfiguration() override;
        void SetConfiguration(const WheelOdometrySensorConfiguration& configuration) override;
        ROS2OdometryCovariance GetPoseCovariance() override;
        void SetPoseCovariance(const ROS2OdometryCovariance& covariance) override;
        ROS2OdometryCovariance GetTwistCovariance() override;
        void SetTwistCovariance(const ROS2OdometryCovariance& covariance) override;

        //////////////////////////////////////////////////////////////////////////

        std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> m_odometryPublisher;
        nav_msgs::msg::Odometry m_odometryMsg;
        AZ::Vector3 m_robotPose = AZ::Vector3::CreateZero();
        AZ::Quaternion m_robotRotation = AZ::Quaternion::CreateIdentity();
        WheelOdometrySensorConfiguration m_odometryConfiguration;

        void OnOdometryEvent();
        void OnPhysicsEvent(float physicsDeltaTime);
    };
} // namespace ROS2Sensors
