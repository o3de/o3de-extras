/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ROS2OdometrySensorComponent.h"
#include <AzCore/Component/ComponentApplicationBus.h>
#include <AzCore/Component/Entity.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>
#include <AzFramework/Physics/RigidBodyBus.h>
#include <AzFramework/Physics/SimulatedBodies/RigidBody.h>
#include <ROS2/Communication/FlexiblePublisher.h>
#include <ROS2/Frame/ROS2FrameComponent.h>
#include <ROS2/ROS2Bus.h>
#include <ROS2/Utilities/ROS2Conversions.h>
#include <ROS2/Utilities/ROS2Names.h>
#include <memory>

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
        required.push_back(AZ_CRC_CE("PhysicsDynamicRigidBodyService"));
        required.push_back(AZ_CRC_CE("ROS2Frame"));
    }

    void ROS2OdometrySensorComponent::SetupRefreshLoop()
    {
        InstallPhysicalCallback();
    }

    void ROS2OdometrySensorComponent::OnPhysicsInitialization(AzPhysics::SceneHandle sceneHandle)
    {
        AzPhysics::RigidBody* rigidBody = nullptr;
        AZ::EntityId entityId = GetEntityId();
        Physics::RigidBodyRequestBus::EventResult(rigidBody, entityId, &Physics::RigidBodyRequests::GetRigidBody);
        AZ_Assert(rigidBody, "Entity %s does not have rigid body.", entityId.ToString().c_str());

        m_bodyHandle = rigidBody->m_bodyHandle;

        auto* sceneInterface = AZ::Interface<AzPhysics::SceneInterface>::Get();
        auto* simulatedBodyPtr = sceneInterface->GetSimulatedBodyFromHandle(sceneHandle, m_bodyHandle);
        auto rigidbodyPtr = azrtti_cast<AzPhysics::RigidBody*>(simulatedBodyPtr);
        AZ_Assert(rigidbodyPtr, "Requested simulated body is not a rigid body");
        m_initialTransform = rigidbodyPtr->GetTransform();
    }

    void ROS2OdometrySensorComponent::OnPhysicsSimulationFinished(AzPhysics::SceneHandle sceneHandle, float deltaTime)
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
    void ROS2OdometrySensorComponent::Activate()
    {
        // "odom" is globally fixed frame for all robots, no matter the namespace
        m_odometryMsg.header.frame_id = ROS2Names::GetNamespacedName(GetNamespace(), "odom").c_str();
        m_odometryMsg.child_frame_id = GetFrameID().c_str();
        AZ_Assert(m_sensorConfiguration.m_publishersConfigurations.size() == 1, "Invalid configuration of publishers for Odometry sensor");

        m_odometryPublisher = std::make_shared<FlexiblePublisher<nav_msgs::msg::Odometry>>(
            m_sensorConfiguration.m_publishersConfigurations[Internal::kOdometryMsgType], GetNamespace(), GetEntityId(), "Odometry sensor");

        ROS2SensorComponent::Activate();
    }

    void ROS2OdometrySensorComponent::Deactivate()
    {
        ROS2SensorComponent::Deactivate();
        m_odometryPublisher.reset();
    }
} // namespace ROS2
