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
#include <ROS2/Sensor/ROS2SensorComponentBase.h>
#include <rclcpp/publisher.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "ImuSensorConfiguration.h"

namespace ROS2
{
    //! An IMU (Inertial Measurement Unit) sensor Component.
    //! IMUs typically include gyroscopes, accelerometers and magnetometers. This component encapsulates data
    //! acquisition and its publishing to ROS2 ecosystem. IMU Component requires ROS2FrameComponent.
    class ROS2ImuSensorComponent : public ROS2SensorComponentBase<PhysicsBasedSource>
    {
    public:
        AZ_COMPONENT(ROS2ImuSensorComponent, "{502A955E-7742-4E23-AD77-5E4063739DCA}", SensorBaseType);
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
        std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Imu>> m_imuPublisher;
        sensor_msgs::msg::Imu m_imuMsg;
        AZ::Vector3 m_previousLinearVelocity = AZ::Vector3::CreateZero();

        AZ::Vector3 m_acceleration{ 0 };
        AZStd::deque<AZ::Vector3> m_filterAcceleration;
        AZStd::deque<AZ::Vector3> m_filterAngularVelocity;

        ImuSensorConfiguration m_imuConfiguration;

        AZ::Matrix3x3 m_orientationCovariance = AZ::Matrix3x3::CreateZero();
        AZ::Matrix3x3 m_angularVelocityCovariance = AZ::Matrix3x3::CreateZero();
        AZ::Matrix3x3 m_linearAccelerationCovariance = AZ::Matrix3x3::CreateZero();

    private:
        void OnPhysicsEvent(AzPhysics::SceneHandle sceneHandle);

        void OnImuEvent(float imuDeltaTime, AzPhysics::SceneHandle sceneHandle, float physicsDeltaTime);

        AZ::Matrix3x3 ToDiagonalCovarianceMatrix(const AZ::Vector3& variance);

        // Handle to the simulated physical body
        AzPhysics::SimulatedBodyHandle m_bodyHandle = AzPhysics::InvalidSimulatedBodyHandle;
    };
} // namespace ROS2
