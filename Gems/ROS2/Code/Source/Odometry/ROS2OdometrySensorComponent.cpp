/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ROS2OdometrySensorComponent.h"
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>
#include <AzFramework/Physics/RigidBodyBus.h>
#include <AzFramework/Physics/SimulatedBodies/RigidBody.h>
#include <ROS2/Frame/ROS2FrameComponent.h>
#include <ROS2/ROS2Bus.h>
#include <ROS2/Utilities/ROS2Conversions.h>
#include <ROS2/Utilities/ROS2Names.h>
#include <ROS2/ROS2GemUtilities.h>

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
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"));
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

    void ROS2OdometrySensorComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("PhysicsRigidBodyService"));
        required.push_back(AZ_CRC_CE("ROS2Frame"));
    }

    // ROS2SensorComponent overrides ...
    void ROS2OdometrySensorComponent::SetupRefreshLoop()
    {
        InstallPhysicalCallback(m_entity->GetId());
    }

    void ROS2OdometrySensorComponent::OnPhysicsInitialization(AzPhysics::SceneHandle sceneHandle)
    {
        if (m_bodyHandle != AzPhysics::InvalidSimulatedBodyHandle)
        {
            auto* sceneInterface = AZ::Interface<AzPhysics::SceneInterface>::Get();
            auto* simulatedBodyPtr = sceneInterface->GetSimulatedBodyFromHandle(sceneHandle, m_bodyHandle);
            auto rigidbodyPtr = azrtti_cast<AzPhysics::RigidBody*>(simulatedBodyPtr);
            AZ_Assert(rigidbodyPtr, "Requested simulated body is not a rigid body");
            m_initialTransform = rigidbodyPtr->GetTransform();
        }
    }

    void ROS2OdometrySensorComponent::OnPhysicsSimulationFinished(AzPhysics::SceneHandle sceneHandle, float deltaTime)
    {
        if (m_odometryPublisher)
        {
            auto* sceneInterface = AZ::Interface<AzPhysics::SceneInterface>::Get();
            auto* simulatedBodyPtr = sceneInterface->GetSimulatedBodyFromHandle(sceneHandle, m_bodyHandle);
            auto rigidbodyPtr = azrtti_cast<AzPhysics::RigidBody*>(simulatedBodyPtr);
            AZ_Assert(rigidbodyPtr, "Requested simulated body is not a rigid body");

            const auto transform = rigidbodyPtr->GetTransform().GetInverse();
            const auto localAngular = transform.TransformVector(rigidbodyPtr->GetAngularVelocity());
            const auto localLinear = transform.TransformVector(rigidbodyPtr->GetLinearVelocity());

            m_odometryMsg.header.stamp = ROS2Interface::Get()->GetROSTimestamp();
            m_odometryMsg.twist.twist.linear = ROS2Conversions::ToROS2Vector3(localLinear);
            m_odometryMsg.twist.twist.angular = ROS2Conversions::ToROS2Vector3(localAngular);

            const auto odometry = m_initialTransform.GetInverse() * rigidbodyPtr->GetTransform();

            if (IsPublicationDeadline(deltaTime))
            {
                m_odometryMsg.pose.pose = ROS2Conversions::ToROS2Pose(odometry);
                m_odometryPublisher->publish(m_odometryMsg);
            }
        }
    }
    void ROS2OdometrySensorComponent::Activate()
    {
        if (ROS2::Utils::IsAutonomousOrNonMultiplayer(GetEntity()))
        {
            // "odom" is globally fixed frame for all robots, no matter the namespace
            m_odometryMsg.header.frame_id = ROS2Names::GetNamespacedName(GetNamespace(), "odom").c_str();
            m_odometryMsg.child_frame_id = GetFrameID().c_str();
            auto ros2Node = ROS2Interface::Get()->GetNode();
            AZ_Assert(m_sensorConfiguration.m_publishersConfigurations.size() == 1, "Invalid configuration of publishers for Odometry sensor");

            const auto publisherConfig = m_sensorConfiguration.m_publishersConfigurations[Internal::kOdometryMsgType];
            const auto fullTopic = ROS2Names::GetNamespacedName(GetNamespace(), publisherConfig.m_topic);
            m_odometryPublisher = ros2Node->create_publisher<nav_msgs::msg::Odometry>(fullTopic.data(), publisherConfig.GetQoS());

            ROS2SensorComponent::Activate();
        }
    }

    void ROS2OdometrySensorComponent::Deactivate()
    {
        if (m_odometryPublisher)
        {
            ROS2SensorComponent::Deactivate();
            m_odometryPublisher.reset();
        }
    }
} // namespace ROS2
