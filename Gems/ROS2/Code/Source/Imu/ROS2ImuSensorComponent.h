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
#include <ROS2/Sensor/ROS2SensorComponent.h>
#include <Utilities/PhysicsCallbackHandler.h>
#include <rclcpp/publisher.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace ROS2
{
    //! An IMU (Inertial Measurement Unit) sensor Component.
    //! IMUs typically include gyroscopes, accelerometers and magnetometers. This component encapsulates data
    //! acquisition and its publishing to ROS2 ecosystem. IMU Component requires ROS2FrameComponent.
    class ROS2ImuSensorComponent
        : public ROS2SensorComponent
        , public ROS2::Utils::PhysicsCallbackHandler
    {
    public:
        AZ_COMPONENT(ROS2ImuSensorComponent, "{502A955E-7742-4E23-AD77-5E4063739DCA}", ROS2SensorComponent);
        ROS2ImuSensorComponent();
        ~ROS2ImuSensorComponent() = default;
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void Reflect(AZ::ReflectContext* context);
        //////////////////////////////////////////////////////////////////////////
        // Component overrides
        void Activate() override;
        void Deactivate() override;
        //////////////////////////////////////////////////////////////////////////

    private:
        static constexpr int m_minFilterSize = 1;

        //! Length of filter that removes numerical noise
        int m_filterSize{ 10 };
<<<<<<< HEAD
        int m_minFilterSize{ 0 };
        int m_maxFilterSize{ 200 };
=======
>>>>>>> 4559cef (Add private limits struct)

        //! Include gravity acceleration
        bool m_includeGravity{ true };

        //! Measure also absolute rotation
        bool m_absoluteRotation{ true };

        std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Imu>> m_imuPublisher;
        sensor_msgs::msg::Imu m_imuMsg;
        AZ::Vector3 m_previousLinearVelocity = AZ::Vector3::CreateZero();

        AZ::Vector3 m_acceleration{ 0 };
        AZStd::deque<AZ::Vector3> m_filterAcceleration;
        AZStd::deque<AZ::Vector3> m_filterAngularVelocity;

        AZ::Vector3 m_orientationVariance = AZ::Vector3::CreateZero();
        AZ::Vector3 m_angularVelocityVariance = AZ::Vector3::CreateZero();
        AZ::Vector3 m_linearAccelerationVariance = AZ::Vector3::CreateZero();


        AZ::Matrix3x3 m_orientationCovariance = AZ::Matrix3x3::CreateZero();
        AZ::Matrix3x3 m_angularVelocityCovariance = AZ::Matrix3x3::CreateZero();
        AZ::Matrix3x3 m_linearAccelerationCovariance = AZ::Matrix3x3::CreateZero();

    private:
        // ROS2SensorComponent overrides ...
        void SetupRefreshLoop() override;

        // ROS2::Utils::PhysicsCallbackHandler overrides ...
        void OnPhysicsSimulationFinished(AzPhysics::SceneHandle sceneHandle, float deltaTime) override;

        AZ::Matrix3x3 ToDiagonalCovarianceMatrix(const AZ::Vector3& variance);
    };
} // namespace ROS2
