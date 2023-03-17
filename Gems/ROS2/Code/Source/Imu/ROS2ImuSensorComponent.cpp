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
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<ROS2ImuSensorComponent, ROS2SensorComponent>()->Version(1)->Field(
                "filterSize", &ROS2ImuSensorComponent::m_filterSize);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<ROS2ImuSensorComponent>("ROS2 Imu Sensor", "Imu sensor component")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &ROS2ImuSensorComponent::m_filterSize, "Filter Length", "Filter Length")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"));
            }
        }
    }

    ROS2ImuSensorComponent::ROS2ImuSensorComponent()
    {
        const AZStd::string msgType = Internal::kImuMsgType;
        TopicConfiguration pc(msgType, "imu");
        m_sensorConfiguration.m_publishersConfigurations.insert(AZStd::make_pair(msgType, pc));
    }

    void ROS2ImuSensorComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("PhysicsRigidBodyService"));
        required.push_back(AZ_CRC_CE("ROS2Frame"));
    }

    void ROS2ImuSensorComponent::Activate()
    {
        auto ts = ROS2Interface::Get()->GetROSTimestamp();
        m_time = ts.sec + ts.nanosec / 1e9;
        auto ros2Node = ROS2Interface::Get()->GetNode();
        AZ_Assert(m_sensorConfiguration.m_publishersConfigurations.size() == 1, "Invalid configuration of publishers for IMU sensor");

        m_imuMsg.header.frame_id = ROS2Names::GetNamespacedName(GetNamespace(), "imu").c_str();

        const auto publisherConfig = m_sensorConfiguration.m_publishersConfigurations[Internal::kImuMsgType];
        const auto fullTopic = ROS2Names::GetNamespacedName(GetNamespace(), publisherConfig.m_topic);
        m_imuPublisher = ros2Node->create_publisher<sensor_msgs::msg::Imu>(fullTopic.data(), publisherConfig.GetQoS());

        m_simulatedBodiesEventHandler = AzPhysics::SceneEvents::OnSceneActiveSimulatedBodiesEvent::Handler(
            [this](AzPhysics::SceneHandle sceneHandle, const AzPhysics::SimulatedBodyHandleList& activeBodyList, float deltaTime)
            {
                if (m_bodyHandle == AzPhysics::InvalidSimulatedBodyHandle)
                {
                    AzPhysics::RigidBody* rigidBody = nullptr;
                    Physics::RigidBodyRequestBus::EventResult(rigidBody, m_entity->GetId(), &Physics::RigidBodyRequests::GetRigidBody);
                    m_bodyHandle = rigidBody->m_bodyHandle;
                }

                auto* sceneInterface = AZ::Interface<AzPhysics::SceneInterface>::Get();
                const auto gravity = sceneInterface->GetGravity(sceneHandle);
                auto* body = sceneInterface->GetSimulatedBodyFromHandle(sceneHandle, m_bodyHandle);
                auto rigidbody = azrtti_cast<AzPhysics::RigidBody*>(body);
                AZ_Assert(rigidbody, "Requested simulated body is not a rigid body");
                auto inv = rigidbody->GetTransform().GetInverse();
                const auto linearVelocity = inv.TransformVector(rigidbody->GetLinearVelocity());
                m_filter.push_back(linearVelocity);
                if (m_filter.size() > m_filterSize)
                {
                    m_filter.pop_front();
                }
                if (IsPublicationDeadline(deltaTime, deltaTime))
                {
                    const AZ::Vector3 linearVelocityFilter =
                        AZStd::accumulate(m_filter.begin(), m_filter.end(), AZ::Vector3{ 0 }) / m_filter.size();

                    auto acc = (linearVelocityFilter - m_previousLinearVelocity) / deltaTime;
                    auto angularVelocity = inv.TransformVector(rigidbody->GetAngularVelocity());
                    m_previousLinearVelocity = linearVelocityFilter;
                    m_acceleration = -acc + inv.TransformVector(gravity) + angularVelocity.Cross(linearVelocityFilter);
                    m_imuMsg.linear_acceleration = ROS2Conversions::ToROS2Vector3(m_acceleration);
                    m_imuMsg.angular_velocity = ROS2Conversions::ToROS2Vector3(angularVelocity);
                    const float timeStamp = m_time - deltaTime * m_filter.size() / 2;
                    m_time += deltaTime;
                    m_imuMsg.header.stamp.sec = static_cast<int32_t>(timeStamp);
                    m_imuMsg.header.stamp.nanosec = static_cast<uint32_t>((timeStamp - AZStd::floor(timeStamp)) * 1e9);
                    this->m_imuPublisher->publish(m_imuMsg);
                }
            });

        auto* sceneInterface = AZ::Interface<AzPhysics::SceneInterface>::Get();
        AzPhysics::SceneHandle sceneHandle = sceneInterface->GetSceneHandle(AzPhysics::DefaultPhysicsSceneName);
        sceneInterface->RegisterSceneActiveSimulatedBodiesHandler(sceneHandle, m_simulatedBodiesEventHandler);
    }

    void ROS2ImuSensorComponent::Deactivate()
    {
        m_bodyHandle = AzPhysics::InvalidSimulatedBodyHandle;
        m_imuPublisher.reset();
    }

} // namespace ROS2
