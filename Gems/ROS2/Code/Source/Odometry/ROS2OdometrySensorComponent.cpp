/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <AzFramework/Physics/PhysicsScene.h>
#include <AzFramework/Physics/RigidBodyBus.h>
#include <AzFramework/Physics/SimulatedBodies/RigidBody.h>
#include "ROS2OdometrySensorComponent.h"
#include <ROS2/Utilities/ROS2Conversions.h>
#include <ROS2/Utilities/ROS2Names.h>

namespace ROS2
{
    namespace
    {
        const char* OdometryMsgType = "nav_msgs::msg::Odometry";
    }

    void ROS2OdometrySensorComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<ROS2OdometrySensorComponent, SensorBaseType>()->Version(2);

            if (auto* editContext = serialize->GetEditContext())
            {
                editContext->Class<ROS2OdometrySensorComponent>("ROS2 Odometry Sensor", "Odometry sensor component")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->Attribute(AZ::Edit::Attributes::Icon, "Editor/Icons/Components/ROS2OdometrySensor.svg")
                    ->Attribute(AZ::Edit::Attributes::ViewportIcon, "Editor/Icons/Components/Viewport/ROS2OdometrySensor.svg");
            }
        }
    }

    ROS2OdometrySensorComponent::ROS2OdometrySensorComponent()
        : m_initialTransform(AZ::Transform::CreateIdentity())
    {
        TopicConfiguration tc;
        const AZStd::string type = OdometryMsgType;
        tc.m_type = type;
        tc.m_topic = "odom";
        m_sensorConfiguration.m_frequency = 10;
        m_sensorConfiguration.m_publishersConfigurations.insert(AZStd::make_pair(type, tc));
    }

    void ROS2OdometrySensorComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("PhysicsDynamicRigidBodyService"));
        required.push_back(AZ_CRC_CE("ROS2Frame"));
    }

    void ROS2OdometrySensorComponent::OnOdometryEvent(AzPhysics::SceneHandle sceneHandle)
    {
        if (m_bodyHandle == AzPhysics::InvalidSimulatedBodyHandle)
        {
            AzPhysics::RigidBody* rigidBody = nullptr;
            AZ::EntityId entityId = GetEntityId();
            Physics::RigidBodyRequestBus::EventResult(rigidBody, entityId, &Physics::RigidBodyRequests::GetRigidBody);
            AZ_Assert(rigidBody, "Entity %s does not have rigid body.", entityId.ToString().c_str());

            m_bodyHandle = rigidBody->m_bodyHandle;

            auto* sceneInterface = AZ::Interface<AzPhysics::SceneInterface>::Get();
            AZ_Assert(sceneInterface, "Requested scene interface is missing");

            auto* simulatedBodyPtr = sceneInterface->GetSimulatedBodyFromHandle(sceneHandle, m_bodyHandle);
            auto* rigidbodyPtr = azrtti_cast<AzPhysics::RigidBody*>(simulatedBodyPtr);
            AZ_Assert(rigidbodyPtr, "Requested simulated body is not a rigid body");
            m_initialTransform = rigidbodyPtr->GetTransform();
        }

        auto* sceneInterface = AZ::Interface<AzPhysics::SceneInterface>::Get();
        AZ_Assert(sceneInterface, "Requested scene interface is missing");

        auto* simulatedBodyPtr = sceneInterface->GetSimulatedBodyFromHandle(sceneHandle, m_bodyHandle);
        auto* rigidbodyPtr = azrtti_cast<AzPhysics::RigidBody*>(simulatedBodyPtr);
        AZ_Assert(rigidbodyPtr, "Requested simulated body is not a rigid body");

        const auto transform = rigidbodyPtr->GetTransform().GetInverse();
        const auto localAngular = transform.TransformVector(rigidbodyPtr->GetAngularVelocity());
        const auto localLinear = transform.TransformVector(rigidbodyPtr->GetLinearVelocity());

        m_odometryMsg.header.stamp = ROS2Interface::Get()->GetROSTimestamp();
        m_odometryMsg.twist.twist.linear = ROS2Conversions::ToROS2Vector3(localLinear);
        m_odometryMsg.twist.twist.angular = ROS2Conversions::ToROS2Vector3(localAngular);

        const auto odometry = m_initialTransform.GetInverse() * rigidbodyPtr->GetTransform();

        m_odometryMsg.pose.pose = ROS2Conversions::ToROS2Pose(odometry);
        m_odometryPublisher->publish(m_odometryMsg);
    }
    void ROS2OdometrySensorComponent::Activate()
    {
        // "odom" is globally fixed frame for all robots, no matter the namespace
        m_odometryMsg.header.frame_id = ROS2Names::GetNamespacedName(GetNamespace(), "odom").c_str();
        m_odometryMsg.child_frame_id = GetFrameID().c_str();
        auto ros2Node = ROS2Interface::Get()->GetNode();
        AZ_Assert(m_sensorConfiguration.m_publishersConfigurations.size() == 1, "Invalid configuration of publishers for Odometry sensor");

        const auto publisherConfig = m_sensorConfiguration.m_publishersConfigurations[OdometryMsgType];
        const auto fullTopic = ROS2Names::GetNamespacedName(GetNamespace(), publisherConfig.m_topic);
        m_odometryPublisher = ros2Node->create_publisher<nav_msgs::msg::Odometry>(fullTopic.data(), publisherConfig.GetQoS());

        StartSensor(
            m_sensorConfiguration.m_frequency,
            [this]([[maybe_unused]] float odomDeltaTime, AzPhysics::SceneHandle sceneHandle, [[maybe_unused]] float physicsDeltaTime)
            {
                if (!m_sensorConfiguration.m_publishingEnabled)
                {
                    return;
                }
                OnOdometryEvent(sceneHandle);
            });
    }

    void ROS2OdometrySensorComponent::Deactivate()
    {
        StopSensor();
        m_odometryPublisher.reset();
    }
} // namespace ROS2
