/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/Math/Matrix3x3.h>
#include <AzCore/Math/Transform.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzFramework/Physics/Common/PhysicsEvents.h>
#include <AzFramework/Physics/PhysicsSystem.h>
#include <ROS2/Sensor/Events/PhysicsBasedSource.h>
#include <ROS2Sensors/Imu/ImuConfigurationRequestBus.h>
#include <ROS2Sensors/Imu/ImuSensorConfiguration.h>
#include <ROS2Sensors/Sensor/ROS2SensorComponentBase.h>
#include <rclcpp/publisher.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace ROS2Sensors
{
    //! An IMU (Inertial Measurement Unit) sensor Component.
    //! IMUs typically include gyroscopes, accelerometers and magnetometers. This component encapsulates data
    //! acquisition and its publishing to ROS2 ecosystem. IMU Component requires ROS2FrameComponent.
    class ROS2ImuSensorComponent
        : public ROS2SensorComponentBase<ROS2::PhysicsBasedSource>
        , protected ImuConfigurationRequestBus::Handler
    {
    public:
        AZ_COMPONENT(ROS2ImuSensorComponent, ROS2Sensors::ROS2ImuSensorComponentTypeId, SensorBaseType);
        ROS2ImuSensorComponent();
        ROS2ImuSensorComponent(const SensorConfiguration& sensorConfiguration, const ImuSensorConfiguration& imuConfiguration);
        ~ROS2ImuSensorComponent() = default;
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void Reflect(AZ::ReflectContext* context);
        //////////////////////////////////////////////////////////////////////////
        // Component overrides
        void Activate() override;
        void Deactivate() override;
        //////////////////////////////////////////////////////////////////////////

    private:
        //////////////////////////////////////////////////////////////////////////
        // ImuConfigurationRequest::Handler overrides
        void SetConfiguration(const ImuSensorConfiguration& configuration) override;
        const ImuSensorConfiguration GetConfiguration() const override;
        int GetFilterSize() const override;
        void SetFilterSize(int filterSize) override;
        int GetMinFilterSize() const override;
        void SetMinFilterSize(int minFilterSize) override;
        int GetMaxFilterSize() const override;
        void SetMaxFilterSize(int maxFilterSize) override;
        bool GetIncludeGravity() const override;
        void SetIncludeGravity(bool includeGravity) override;
        bool GetAbsoluteRotation() const override;
        void SetAbsoluteRotation(bool absoluteRotation) override;
        AZ::Vector3 GetOrientationVariance() const override;
        void SetOrientationVariance(const AZ::Vector3& orientationVariance) override;
        AZ::Vector3 GetAngularVelocityVariance() const override;
        void SetAngularVelocityVariance(const AZ::Vector3& angularVelocityVariance) override;
        AZ::Vector3 GetLinearAccelerationVariance() const override;
        void SetLinearAccelerationVariance(const AZ::Vector3& linearAccelerationVariance) override;
        //////////////////////////////////////////////////////////////////////////

        void ConfigureSensor();

        ImuSensorConfiguration m_imuConfiguration;

        std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Imu>> m_imuPublisher;
        sensor_msgs::msg::Imu m_imuMsg;
        AZ::Vector3 m_previousLinearVelocity = AZ::Vector3::CreateZero();

        AZ::Vector3 m_acceleration{ 0 };
        AZStd::deque<AZ::Vector3> m_filterAcceleration;
        AZStd::deque<AZ::Vector3> m_filterAngularVelocity;

        AZ::Matrix3x3 m_orientationCovariance = AZ::Matrix3x3::CreateZero();
        AZ::Matrix3x3 m_angularVelocityCovariance = AZ::Matrix3x3::CreateZero();
        AZ::Matrix3x3 m_linearAccelerationCovariance = AZ::Matrix3x3::CreateZero();

        void OnPhysicsEvent(AzPhysics::SceneHandle sceneHandle);

        void OnImuEvent(float imuDeltaTime, AzPhysics::SceneHandle sceneHandle, float physicsDeltaTime);

        AZ::Matrix3x3 ToDiagonalCovarianceMatrix(const AZ::Vector3& variance);

        // Handle to the simulated physical body
        AzPhysics::SimulatedBodyHandle m_bodyHandle = AzPhysics::InvalidSimulatedBodyHandle;
    };
} // namespace ROS2Sensors
