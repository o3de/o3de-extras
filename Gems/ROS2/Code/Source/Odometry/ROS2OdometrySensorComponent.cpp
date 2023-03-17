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
#include <ROS2/Frame/ROS2FrameComponent.h>
#include <ROS2/ROS2Bus.h>
#include <ROS2/Utilities/ROS2Conversions.h>
#include <ROS2/Utilities/ROS2Names.h>
#include <AzFramework/Physics/SimulatedBodies/RigidBody.h>

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
        m_onSceneSimulationEvent = AzPhysics::SceneEvents::OnSceneSimulationFinishHandler(
            [this]([[maybe_unused]]AzPhysics::SceneHandle sceneHandle, float deltaTime)
            {
                if (m_rigidBodyPtr == nullptr)
                {
                    Physics::RigidBodyRequestBus::EventResult(m_rigidBodyPtr, m_entity->GetId(), &Physics::RigidBodyRequests::GetRigidBody);
                    m_initialTransform = m_rigidBodyPtr->GetTransform();
                    AZ_Assert(m_rigidBodyPtr, "NoRigdBody");
                }
                AZ_Assert(m_rigidBodyPtr, "NoRigdBody");

                const auto transform = m_rigidBodyPtr->GetTransform().GetInverse();
                const auto localAngular = transform.TransformVector(m_rigidBodyPtr->GetAngularVelocity());
                const auto localLinear = transform.TransformVector(m_rigidBodyPtr->GetLinearVelocity());

                m_odometryMsg.header.stamp = ROS2Interface::Get()->GetROSTimestamp();
                m_odometryMsg.twist.twist.linear = ROS2Conversions::ToROS2Vector3(localLinear);
                m_odometryMsg.twist.twist.angular = ROS2Conversions::ToROS2Vector3(localAngular);

                if (m_sensorConfiguration.m_frequency>0)
                {
                    const auto odometry = m_initialTransform.GetInverse() * m_rigidBodyPtr->GetTransform();
                    m_robotPose = odometry.GetTranslation();
                    m_robotRotation = odometry.GetRotation();
                }
                if (IsPublicationDeadline(deltaTime))
                {
                    m_odometryMsg.pose.pose.position = ROS2Conversions::ToROS2Point(m_robotPose);
                    m_odometryMsg.pose.pose.orientation = ROS2Conversions::ToROS2Quaternion(m_robotRotation);
                    m_odometryPublisher->publish(m_odometryMsg);
                }
            });

        auto* sceneInterface = AZ::Interface<AzPhysics::SceneInterface>::Get();
        AzPhysics::SceneHandle sceneHandle = sceneInterface->GetSceneHandle(AzPhysics::DefaultPhysicsSceneName);
        sceneInterface->RegisterSceneSimulationFinishHandler(sceneHandle, m_onSceneSimulationEvent);
    }

    void ROS2OdometrySensorComponent::Activate()
    {

        m_rigidBodyPtr = nullptr;
        // "odom" is globally fixed frame for all robots, no matter the namespace
        m_odometryMsg.header.frame_id = ROS2Names::GetNamespacedName(GetNamespace(), "odom").c_str();
        m_odometryMsg.child_frame_id = GetFrameID().c_str();
        auto ros2Node = ROS2Interface::Get()->GetNode();
        AZ_Assert(m_sensorConfiguration.m_publishersConfigurations.size() == 1, "Invalid configuration of publishers for Odometry sensor");

        const auto publisherConfig = m_sensorConfiguration.m_publishersConfigurations[Internal::kOdometryMsgType];
        const auto fullTopic = ROS2Names::GetNamespacedName(GetNamespace(), publisherConfig.m_topic);
        m_odometryPublisher = ros2Node->create_publisher<nav_msgs::msg::Odometry>(fullTopic.data(), publisherConfig.GetQoS());

        m_robotPose = AZ::Vector3 {0};
        m_robotRotation = AZ::Quaternion {0,0,0,1};
        ROS2SensorComponent::Activate();

    }

    void ROS2OdometrySensorComponent::Deactivate()
    {
        m_onSceneSimulationEvent.Disconnect();
        ROS2SensorComponent::Deactivate();
        m_odometryPublisher.reset();
    }
} // namespace ROS2
