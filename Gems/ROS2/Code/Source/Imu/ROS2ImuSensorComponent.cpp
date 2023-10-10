/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ROS2ImuSensorComponent.h"
#include <ROS2/Frame/ROS2FrameComponent.h>
#include <ROS2/ROS2Bus.h>
#include <ROS2/Utilities/ROS2Conversions.h>
#include <ROS2/Utilities/ROS2Names.h>

#include <AzFramework/Physics/SimulatedBodies/RigidBody.h>
#include <Source/RigidBodyComponent.h>

#include <AzCore/Component/Entity.h>
#include <AzCore/Script/ScriptTimePoint.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>
#include <AzCore/std/numeric.h>
#include <AzCore/std/smart_ptr/make_shared.h>

namespace ROS2
{
    namespace Internal
    {
        const char* kImuMsgType = "sensor_msgs::msg::Imu";
    }

    void ROS2ImuSensorComponent::Reflect(AZ::ReflectContext* context)
    {
        ImuSensorConfiguration::Reflect(context);

        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<ROS2ImuSensorComponent, SensorBaseType>()->Version(3)->Field(
                "imuSensorConfiguration", &ROS2ImuSensorComponent::m_imuConfiguration);

            if (auto* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<ROS2ImuSensorComponent>("ROS2 Imu Sensor", "Imu sensor component")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->Attribute(AZ::Edit::Attributes::Icon, "Editor/Icons/Components/ROS2ImuSensor.svg")
                    ->Attribute(AZ::Edit::Attributes::ViewportIcon, "Editor/Icons/Components/Viewport/ROS2ImuSensor.svg")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ROS2ImuSensorComponent::m_imuConfiguration,
                        "Imu sensor configuration",
                        "Imu sensor configuration")
                    ->Attribute(AZ::Edit::Attributes::Visibility, AZ::Edit::PropertyVisibility::ShowChildrenOnly);
            }
        }
    }

    ROS2ImuSensorComponent::ROS2ImuSensorComponent()
    {
        const AZStd::string type = Internal::kImuMsgType;
        TopicConfiguration pc;
        pc.m_type = type;
        pc.m_topic = "imu";
        m_sensorConfiguration.m_frequency = 50;
        m_sensorConfiguration.m_publishersConfigurations.insert(AZStd::make_pair(type, pc));
    }

    ROS2ImuSensorComponent::ROS2ImuSensorComponent(
        const SensorConfiguration& sensorConfiguration, const ImuSensorConfiguration& imuConfiguration)
        : m_imuConfiguration(imuConfiguration)
    {
        m_sensorConfiguration = sensorConfiguration;
    }

    void ROS2ImuSensorComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("PhysicsDynamicRigidBodyService"));
        required.push_back(AZ_CRC_CE("ROS2Frame"));
    }

    void ROS2ImuSensorComponent::Activate()
    {
        auto ros2Node = ROS2Interface::Get()->GetNode();
        AZ_Assert(m_sensorConfiguration.m_publishersConfigurations.size() == 1, "Invalid configuration of publishers for IMU sensor");
        m_imuMsg.header.frame_id = GetFrameID().c_str();
        const auto publisherConfig = m_sensorConfiguration.m_publishersConfigurations[Internal::kImuMsgType];
        const auto fullTopic = ROS2Names::GetNamespacedName(GetNamespace(), publisherConfig.m_topic);
        m_imuPublisher = ros2Node->create_publisher<sensor_msgs::msg::Imu>(fullTopic.data(), publisherConfig.GetQoS());

        m_linearAccelerationCovariance = ToDiagonalCovarianceMatrix(m_imuConfiguration.m_linearAccelerationVariance);
        m_angularVelocityCovariance = ToDiagonalCovarianceMatrix(m_imuConfiguration.m_angularVelocityVariance);
        m_orientationCovariance = ToDiagonalCovarianceMatrix(m_imuConfiguration.m_orientationVariance);

        StartSensor(
            m_sensorConfiguration.m_frequency,
            [this](float imuDeltaTime, AzPhysics::SceneHandle sceneHandle, float physicsDeltaTime)
            {
                if (!m_sensorConfiguration.m_publishingEnabled)
                {
                    return;
                }
                OnImuEvent(imuDeltaTime, sceneHandle, physicsDeltaTime);
            },
            [this]([[maybe_unused]] AzPhysics::SceneHandle sceneHandle, float physicsDeltaTime)
            {
                OnPhysicsEvent(sceneHandle);
            });
    }

    void ROS2ImuSensorComponent::Deactivate()
    {
        StopSensor();
        m_bodyHandle = AzPhysics::InvalidSimulatedBodyHandle;
        m_imuPublisher.reset();
    }

    void ROS2ImuSensorComponent::OnPhysicsEvent(AzPhysics::SceneHandle sceneHandle)
    {
        if (m_bodyHandle == AzPhysics::InvalidSimulatedBodyHandle)
        {
            AzPhysics::RigidBody* rigidBody = nullptr;
            AZ::EntityId entityId = GetEntityId();
            Physics::RigidBodyRequestBus::EventResult(rigidBody, entityId, &Physics::RigidBodyRequests::GetRigidBody);

            if (!rigidBody)
            {
                AZ_Error("ROS2ImuSensorComponent", false, "Entity %s does not have a rigid body - stopping Imu sensor.", entityId.ToString().c_str());
                StopSensor();
                return;
            }

            m_bodyHandle = rigidBody->m_bodyHandle;
        }

        auto* sceneInterface = AZ::Interface<AzPhysics::SceneInterface>::Get();
        auto* body = sceneInterface->GetSimulatedBodyFromHandle(sceneHandle, m_bodyHandle);
        AZ_Assert(body, "Requested simulated body does not exist");
        auto* rigidbody = azrtti_cast<AzPhysics::RigidBody*>(body);
        AZ_Assert(rigidbody, "Requested simulated body is not a rigid body");

        auto inv = rigidbody->GetTransform().GetInverse();
        const auto linearVelocity = inv.TransformVector(rigidbody->GetLinearVelocity());
        m_filterAcceleration.push_back(linearVelocity);
        const auto angularVelocity = inv.TransformVector(rigidbody->GetAngularVelocity());
        m_filterAngularVelocity.push_back(angularVelocity);
        if (m_filterAcceleration.size() > m_imuConfiguration.m_filterSize)
        {
            m_filterAcceleration.pop_front();
            m_filterAngularVelocity.pop_front();
        }
    }

    void ROS2ImuSensorComponent::OnImuEvent([[maybe_unused]] float imuDeltaTime, AzPhysics::SceneHandle sceneHandle, float physicsDeltaTime)
    {
        const AZ::Vector3 linearVelocityFilter =
            AZStd::accumulate(m_filterAcceleration.begin(), m_filterAcceleration.end(), AZ::Vector3{ 0 }) / m_filterAcceleration.size();

        const AZ::Vector3 angularRateFiltered =
            AZStd::accumulate(m_filterAngularVelocity.begin(), m_filterAngularVelocity.end(), AZ::Vector3{ 0 }) /
            m_filterAngularVelocity.size();

        // Physics delta time is used here intentionally - linear velocities are accumulated with that delta (see OnPhysicsEvent method).
        auto acc = (linearVelocityFilter - m_previousLinearVelocity) / imuDeltaTime;

        auto* sceneInterface = AZ::Interface<AzPhysics::SceneInterface>::Get();
        auto* body = sceneInterface->GetSimulatedBodyFromHandle(sceneHandle, m_bodyHandle);
        auto rigidbody = azrtti_cast<AzPhysics::RigidBody*>(body);
        auto inv = rigidbody->GetTransform().GetInverse();

        m_previousLinearVelocity = linearVelocityFilter;
        m_acceleration = acc - angularRateFiltered.Cross(linearVelocityFilter);

        if (m_imuConfiguration.m_includeGravity)
        {
            const auto gravity = sceneInterface->GetGravity(sceneHandle);
            m_acceleration -= inv.TransformVector(gravity);
        }
        m_imuMsg.linear_acceleration = ROS2Conversions::ToROS2Vector3(m_acceleration);
        m_imuMsg.linear_acceleration_covariance = ROS2Conversions::ToROS2Covariance(m_linearAccelerationCovariance);
        m_imuMsg.angular_velocity = ROS2Conversions::ToROS2Vector3(angularRateFiltered);
        m_imuMsg.angular_velocity_covariance = ROS2Conversions::ToROS2Covariance(m_angularVelocityCovariance);

        if (m_imuConfiguration.m_absoluteRotation)
        {
            m_imuMsg.orientation = ROS2Conversions::ToROS2Quaternion(rigidbody->GetTransform().GetRotation());
            m_imuMsg.orientation_covariance = ROS2Conversions::ToROS2Covariance(m_orientationCovariance);
        }
        m_imuMsg.header.stamp = ROS2Interface::Get()->GetROSTimestamp();
        this->m_imuPublisher->publish(m_imuMsg);
    }

    AZ::Matrix3x3 ROS2ImuSensorComponent::ToDiagonalCovarianceMatrix(const AZ::Vector3& variance)
    {
        AZ::Matrix3x3 covarianceMatrix;
        covarianceMatrix.SetElement(0, 0, variance.GetX());
        covarianceMatrix.SetElement(1, 1, variance.GetY());
        covarianceMatrix.SetElement(2, 2, variance.GetZ());
        return covarianceMatrix;
    }

} // namespace ROS2
