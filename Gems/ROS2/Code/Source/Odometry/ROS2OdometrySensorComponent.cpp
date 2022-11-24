/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ROS2OdometrySensorComponent.h"
#include "ROS2/Frame/ROS2FrameComponent.h"
#include "ROS2/ROS2Bus.h"
#include "ROS2/Utilities/ROS2Conversions.h"
#include "ROS2/Utilities/ROS2Names.h"
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>
#include <AzFramework/Physics/RigidBodyBus.h>

namespace ROS2
{
    namespace Internal
    {
        const char* kOdometryMsgType = "nav_msgs::msg::Odometry";
    }

    void ROS2OdometrySensorComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<ROS2OdometrySensorComponent, ROS2SensorComponent>()->Version(1);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<ROS2OdometrySensorComponent>("ROS2 Odometry Sensor", "Odometry sensor component")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"));
            }
        }
    }

    ROS2OdometrySensorComponent::ROS2OdometrySensorComponent()
    {
        TopicConfiguration tc;
        const AZStd::string type = Internal::kOdometryMsgType;
        tc.m_type = type;
        tc.m_topic = "odom";
        m_sensorConfiguration.m_frequency = 10;
        m_sensorConfiguration.m_publishersConfigurations.insert(AZStd::make_pair(type, tc));
    }

    void ROS2OdometrySensorComponent::FrequencyTick()
    {
        auto* ros2Frame = Utils::GetGameOrEditorComponent<ROS2FrameComponent>(GetEntity());
        auto transform = ros2Frame->GetFrameTransform();

        AZ::Vector3 linearVelocity;
        Physics::RigidBodyRequestBus::EventResult(linearVelocity, m_entity->GetId(), &Physics::RigidBodyRequests::GetLinearVelocity);

        linearVelocity = transform.GetInverse().TransformVector(linearVelocity);
        AZ::Vector3 angularVelocity;
        Physics::RigidBodyRequestBus::EventResult(angularVelocity, m_entity->GetId(), &Physics::RigidBodyRequests::GetAngularVelocity);
        angularVelocity = transform.GetInverse().TransformVector(angularVelocity);
        m_odometryMsg.twist.twist.linear = ROS2Conversions::ToROS2Vector3(linearVelocity);
        m_odometryMsg.twist.twist.angular = ROS2Conversions::ToROS2Vector3(angularVelocity);

        auto translation = transform.GetTranslation();
        m_odometryMsg.pose.pose.position.x = translation.GetX();
        m_odometryMsg.pose.pose.position.y = translation.GetY();
        m_odometryMsg.pose.pose.position.z = translation.GetZ();

        m_odometryPublisher->publish(m_odometryMsg);
    }

    void ROS2OdometrySensorComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("PhysicsRigidBodyService"));
    }

    void ROS2OdometrySensorComponent::Activate()
    {
        // "odom" is globally fixed frame for all robots, no matter the namespace
        m_odometryMsg.header.frame_id = ROS2Names::GetNamespacedName(GetNamespace(), "odom").c_str();
        m_odometryMsg.child_frame_id = GetFrameID().c_str();

        ROS2SensorComponent::Activate();
        auto ros2Node = ROS2Interface::Get()->GetNode();
        AZ_Assert(m_sensorConfiguration.m_publishersConfigurations.size() == 1, "Invalid configuration of publishers for Odometry sensor");

        const auto publisherConfig = m_sensorConfiguration.m_publishersConfigurations[Internal::kOdometryMsgType];
        const auto fullTopic = ROS2Names::GetNamespacedName(GetNamespace(), publisherConfig.m_topic);
        m_odometryPublisher = ros2Node->create_publisher<nav_msgs::msg::Odometry>(fullTopic.data(), publisherConfig.GetQoS());
    }

    void ROS2OdometrySensorComponent::Deactivate()
    {
        ROS2SensorComponent::Deactivate();
        m_odometryPublisher.reset();
    }
} // namespace ROS2
