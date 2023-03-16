/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ROS2OldImuSensorComponent.h"
#include <ROS2/Frame/ROS2FrameComponent.h>
#include <ROS2/ROS2Bus.h>
#include <ROS2/Utilities/ROS2Conversions.h>
#include <ROS2/Utilities/ROS2Names.h>

#include <AzCore/Component/Entity.h>
#include <AzCore/Script/ScriptTimePoint.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>
#include <AzCore/std/smart_ptr/make_shared.h>

namespace ROS2
{
    namespace Internal
    {
        const char* kOImuMsgType = "sensor_msgs::msg::OImu";
    }

    void ROS2OldImuSensorComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<ROS2OldImuSensorComponent, ROS2SensorComponent>()->Version(1);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<ROS2OldImuSensorComponent>("ROS2 OImu Sensor", "OImu sensor component")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"));
            }
        }
    }

    ROS2OldImuSensorComponent::ROS2OldImuSensorComponent()
    {
        TopicConfiguration pc;
        const AZStd::string type = Internal::kImuMsgType;
        pc.m_type = type;
        pc.m_topic = "imu";
        m_sensorConfiguration.m_frequency = 10;
        m_sensorConfiguration.m_publishersConfigurations.insert(AZStd::make_pair(type, pc));
    }

    void ROS2OldImuSensorComponent::Activate()
    {
        ROS2SensorComponent::Activate();
        auto ros2Node = ROS2Interface::Get()->GetNode();
        AZ_Assert(m_sensorConfiguration.m_publishersConfigurations.size() == 1, "Invalid configuration of publishers for IMU sensor");

        const auto publisherConfig = m_sensorConfiguration.m_publishersConfigurations[Internal::kImuMsgType];
        const auto fullTopic = ROS2Names::GetNamespacedName(GetNamespace(), publisherConfig.m_topic);
        m_imuPublisher = ros2Node->create_publisher<sensor_msgs::msg::Imu>(fullTopic.data(), publisherConfig.GetQoS());

        InitializeImuMessage();

        m_previousPose = GetCurrentPose();
        m_previousTime = GetCurrentTimeInSec();
    }

    void ROS2OldImuSensorComponent::Deactivate()
    {
        ROS2SensorComponent::Deactivate();
        m_imuPublisher.reset();
    }

    void ROS2OldImuSensorComponent::FrequencyTick()
    {
        const double currentTime = GetCurrentTimeInSec();
        const auto timeDiff = currentTime - m_previousTime;

        const auto currentPose = GetCurrentPose();
        const auto frequency = 1.0 / timeDiff;

        // Calculate angular velocity
        const auto& currentRotation = currentPose.GetRotation();
        const auto& previousRotation = m_previousPose.GetRotation();
        const auto deltaRotation = currentRotation * previousRotation.GetInverseFull();
        AZ::Vector3 axis;
        float angle;
        deltaRotation.ConvertToAxisAngle(axis, angle);
        const auto angularVelocity = frequency * angle * axis;

        // Calculate linear acceleration
        const auto& currentPosition = currentPose.GetTranslation();
        const auto deltaPositions = currentPosition - m_previousPose.GetTranslation();
        const auto linearVelocity = currentPose.GetInverse().TransformVector(deltaPositions) * frequency;
        const auto linearAcceleration = (linearVelocity - m_previousLinearVelocity) * frequency;

        // Store current values
        m_previousTime = currentTime;
        m_previousPose = currentPose;
        m_previousLinearVelocity = linearVelocity;

        // Fill message fields
        m_imuMsg.header.frame_id = GetFrameID().data();
        m_imuMsg.header.stamp = ROS2Interface::Get()->GetROSTimestamp();

        m_imuMsg.angular_velocity = ROS2Conversions::ToROS2Vector3(angularVelocity);
        m_imuMsg.linear_acceleration = ROS2Conversions::ToROS2Vector3(linearAcceleration);

        m_imuPublisher->publish(m_imuMsg);
    }

    void ROS2OldImuSensorComponent::InitializeImuMessage()
    {
        // Set identity orientation
        m_imuMsg.orientation.x = 0.0;
        m_imuMsg.orientation.y = 0.0;
        m_imuMsg.orientation.z = 0.0;
        m_imuMsg.orientation.w = 1.0;

        // Set all the covariances to 0
        for (auto& e : m_imuMsg.orientation_covariance)
        {
            e = 0.0;
        }

        for (auto& e : m_imuMsg.angular_velocity_covariance)
        {
            e = 0.0;
        }

        for (auto& e : m_imuMsg.linear_acceleration_covariance)
        {
            e = 0.0;
        }
    }

    double ROS2OldImuSensorComponent::GetCurrentTimeInSec() const
    {
        AZ::ScriptTimePoint timePoint;
        return timePoint.GetSeconds();
    }

    AZ::Transform ROS2OldImuSensorComponent::GetCurrentPose() const
    {
        auto* ros2Frame = Utils::GetGameOrEditorComponent<ROS2FrameComponent>(GetEntity());
        return ros2Frame->GetFrameTransform();
    }

} // namespace ROS2
