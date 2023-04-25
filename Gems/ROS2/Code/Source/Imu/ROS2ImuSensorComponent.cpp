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
            serialize->Class<ROS2ImuSensorComponent, ROS2SensorComponent>()
                ->Version(1)
                ->Field("FilterSize", &ROS2ImuSensorComponent::m_filterSize)
                ->Field("IncludeGravity", &ROS2ImuSensorComponent::m_includeGravity)
                ->Field("AbsoluteRotation", &ROS2ImuSensorComponent::m_absoluteRotation);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<ROS2ImuSensorComponent>("ROS2 Imu Sensor", "Imu sensor component")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ROS2ImuSensorComponent::m_filterSize,
                        "Filter Length",
                        "Filter Length, large value allows to reduce numeric noise")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ROS2ImuSensorComponent::m_includeGravity,
                        "Include Gravitation",
                        "Enable accelerometer to observe gravity force.")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ROS2ImuSensorComponent::m_absoluteRotation,
                        "Absolute Rotation",
                        "Include Absolute rotation in message.");
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

    void ROS2ImuSensorComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("PhysicsRigidBodyService"));
        required.push_back(AZ_CRC_CE("ROS2Frame"));
    }

    void ROS2ImuSensorComponent::SetupRefreshLoop()
    {
        InstallPhysicalCallback(m_entity->GetId());
    }

    void ROS2ImuSensorComponent::OnPhysicsSimulationFinished(AzPhysics::SceneHandle sceneHandle, float deltaTime)
    {
        auto* sceneInterface = AZ::Interface<AzPhysics::SceneInterface>::Get();
        const auto gravity = sceneInterface->GetGravity(sceneHandle);
        auto* body = sceneInterface->GetSimulatedBodyFromHandle(sceneHandle, m_bodyHandle);
        auto rigidbody = azrtti_cast<AzPhysics::RigidBody*>(body);
        AZ_Assert(rigidbody, "Requested simulated body is not a rigid body");
        auto inv = rigidbody->GetTransform().GetInverse();
        const auto linearVelocity = inv.TransformVector(rigidbody->GetLinearVelocity());
        m_filterAcceleration.push_back(linearVelocity);
        const auto angularVelocity = inv.TransformVector(rigidbody->GetAngularVelocity());
        m_filterAngularVelocity.push_back(angularVelocity);
        if (m_filterAcceleration.size() > m_filterSize)
        {
            m_filterAcceleration.pop_front();
            m_filterAngularVelocity.pop_front();
        }
        if (IsPublicationDeadline(deltaTime))
        {
            const AZ::Vector3 linearVelocityFilter =
                AZStd::accumulate(m_filterAcceleration.begin(), m_filterAcceleration.end(), AZ::Vector3{ 0 }) / m_filterAcceleration.size();

            const AZ::Vector3 angularRateFiltered =
                AZStd::accumulate(m_filterAngularVelocity.begin(), m_filterAngularVelocity.end(), AZ::Vector3{ 0 }) /
                m_filterAngularVelocity.size();

            auto acc = (linearVelocityFilter - m_previousLinearVelocity) / deltaTime;

            m_previousLinearVelocity = linearVelocityFilter;
            m_acceleration = -acc + angularRateFiltered.Cross(linearVelocityFilter);
            if (m_includeGravity)
            {
                m_acceleration += inv.TransformVector(gravity);
            }
            m_imuMsg.linear_acceleration = ROS2Conversions::ToROS2Vector3(m_acceleration);
            m_imuMsg.angular_velocity = ROS2Conversions::ToROS2Vector3(angularRateFiltered);
            if (m_absoluteRotation)
            {
                m_imuMsg.orientation = ROS2Conversions::ToROS2Quaternion(rigidbody->GetTransform().GetRotation());
            }
            m_imuMsg.header.stamp = ROS2Interface::Get()->GetROSTimestamp();
            this->m_imuPublisher->publish(m_imuMsg);
        }
    };

    void ROS2ImuSensorComponent::Activate()
    {
        auto ros2Node = ROS2Interface::Get()->GetNode();
        AZ_Assert(m_sensorConfiguration.m_publishersConfigurations.size() == 1, "Invalid configuration of publishers for IMU sensor");
        m_imuMsg.header.frame_id = GetFrameID().c_str();
        const auto publisherConfig = m_sensorConfiguration.m_publishersConfigurations[Internal::kImuMsgType];
        const auto fullTopic = ROS2Names::GetNamespacedName(GetNamespace(), publisherConfig.m_topic);
        m_imuPublisher = ros2Node->create_publisher<sensor_msgs::msg::Imu>(fullTopic.data(), publisherConfig.GetQoS());
        ROS2SensorComponent::Activate();
    }

    void ROS2ImuSensorComponent::Deactivate()
    {
        RemovePhysicalCallback();
        m_bodyHandle = AzPhysics::InvalidSimulatedBodyHandle;
        m_imuPublisher.reset();
    }

} // namespace ROS2
