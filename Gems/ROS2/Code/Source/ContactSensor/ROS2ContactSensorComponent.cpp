/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ROS2ContactSensorComponent.h"
#include <AzFramework/Physics/Collision/CollisionEvents.h>
#include <AzFramework/Physics/Common/PhysicsSimulatedBody.h>
#include <AzFramework/Physics/PhysicsSystem.h>
#include <ROS2/Frame/ROS2FrameComponent.h>
#include <ROS2/ROS2GemUtilities.h>
#include <ROS2/Utilities/ROS2Conversions.h>
#include <ROS2/Utilities/ROS2Names.h>
#include <geometry_msgs/msg/wrench.hpp>

namespace ROS2
{
    namespace
    {
        constexpr float ContactMaximumSeparation = 0.0001f;
    }

    ROS2ContactSensorComponent::ROS2ContactSensorComponent()
    {
        TopicConfiguration tc;
        AZStd::string type = "gazebo_msgs::msg::ContactsState";
        tc.m_type = type;
        tc.m_topic = "contact_sensor";
        m_sensorConfiguration.m_frequency = 15;
        m_sensorConfiguration.m_publishersConfigurations.insert(AZStd::make_pair(AZStd::move(type), AZStd::move(tc)));
    }

    void ROS2ContactSensorComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<ROS2ContactSensorComponent, SensorBaseType>()->Version(2);

            if (auto* editContext = serialize->GetEditContext())
            {
                editContext->Class<ROS2ContactSensorComponent>("ROS2 Contact Sensor", "Contact detection controller")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->Attribute(AZ::Edit::Attributes::Icon, "Editor/Icons/Components/ROS2ContactSensor.svg")
                    ->Attribute(AZ::Edit::Attributes::ViewportIcon, "Editor/Icons/Components/Viewport/ROS2ContactSensor.svg");
            }
        }
    }

    void ROS2ContactSensorComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("PhysicsColliderService"));
        required.push_back(AZ_CRC_CE("ROS2Frame"));
    }

    void ROS2ContactSensorComponent::Activate()
    {
        m_entityId = GetEntityId();
        AZ::Entity* entity = nullptr;
        AZ::ComponentApplicationBus::BroadcastResult(entity, &AZ::ComponentApplicationRequests::FindEntity, m_entityId);
        m_entityName = entity->GetName();

        auto ros2Node = ROS2Interface::Get()->GetNode();
        AZ_Assert(m_sensorConfiguration.m_publishersConfigurations.size() == 1, "Invalid configuration of publishers for Contact sensor");
        const auto publisherConfig = m_sensorConfiguration.m_publishersConfigurations["gazebo_msgs::msg::ContactsState"];
        const auto fullTopic = ROS2Names::GetNamespacedName(GetNamespace(), publisherConfig.m_topic);
        m_contactsPublisher = ros2Node->create_publisher<gazebo_msgs::msg::ContactsState>(fullTopic.data(), publisherConfig.GetQoS());

        m_onCollisionBeginHandler = AzPhysics::SimulatedBodyEvents::OnCollisionBegin::Handler(
            [this]([[maybe_unused]] AzPhysics::SimulatedBodyHandle bodyHandle, const AzPhysics::CollisionEvent& event)
            {
                AddNewContact(event);
            });

        m_onCollisionPersistHandler = AzPhysics::SimulatedBodyEvents::OnCollisionPersist::Handler(
            [this]([[maybe_unused]] AzPhysics::SimulatedBodyHandle bodyHandle, const AzPhysics::CollisionEvent& event)
            {
                AddNewContact(event);
            });

        m_onCollisionEndHandler = AzPhysics::SimulatedBodyEvents::OnCollisionEnd::Handler(
            [this]([[maybe_unused]] AzPhysics::SimulatedBodyHandle bodyHandle, const AzPhysics::CollisionEvent& event)
            {
                AZStd::lock_guard<AZStd::mutex> lock(m_activeContactsMutex);
                m_activeContacts.erase(event.m_body2->GetEntityId());
            });

        StartSensor(
            m_sensorConfiguration.m_frequency,
            [this](auto&&... args)
            {
                if (!m_sensorConfiguration.m_publishingEnabled)
                {
                    return;
                }
                FrequencyTick();
            });
    }

    void ROS2ContactSensorComponent::Deactivate()
    {
        StopSensor();
        m_activeContacts.clear();
        m_contactsPublisher.reset();
        m_onCollisionBeginHandler.Disconnect();
        m_onCollisionPersistHandler.Disconnect();
        m_onCollisionEndHandler.Disconnect();
    }

    void ROS2ContactSensorComponent::FrequencyTick()
    {
        // Connects the collision handlers if not already connected
        AzPhysics::SystemInterface* physicsSystem = AZ::Interface<AzPhysics::SystemInterface>::Get();
        if (!physicsSystem)
        {
            return;
        }

        if (!m_onCollisionBeginHandler.IsConnected() || !m_onCollisionPersistHandler.IsConnected() ||
            !m_onCollisionEndHandler.IsConnected())
        {
            AZStd::pair<AzPhysics::SceneHandle, AzPhysics::SimulatedBodyHandle> foundBody =
                physicsSystem->FindAttachedBodyHandleFromEntityId(GetEntityId());
            AZ_Warning("Contact Sensor", foundBody.first != AzPhysics::InvalidSceneHandle, "Invalid scene handle")
            if (foundBody.first != AzPhysics::InvalidSceneHandle)
            {
                AzPhysics::SimulatedBodyEvents::RegisterOnCollisionBeginHandler(
                    foundBody.first, foundBody.second, m_onCollisionBeginHandler);
                AzPhysics::SimulatedBodyEvents::RegisterOnCollisionPersistHandler(
                    foundBody.first, foundBody.second, m_onCollisionPersistHandler);
                AzPhysics::SimulatedBodyEvents::RegisterOnCollisionEndHandler(foundBody.first, foundBody.second, m_onCollisionEndHandler);
            }
        }

        // Publishes all contacts
        gazebo_msgs::msg::ContactsState msg;
        const auto* ros2Frame = GetEntity()->FindComponent<ROS2FrameComponent>();
        AZ_Assert(ros2Frame, "Invalid component pointer value");
        msg.header.frame_id = ros2Frame->GetFrameID().data();
        msg.header.stamp = ROS2Interface::Get()->GetROSTimestamp();

        {
            // If there are no active collisions, then there is nothing to send
            AZStd::lock_guard<AZStd::mutex> lock(m_activeContactsMutex);
            if (!m_activeContacts.empty())
            {
                for (auto [id, contact] : m_activeContacts)
                {
                    msg.states.push_back(AZStd::move(contact));
                }
                m_contactsPublisher->publish(AZStd::move(msg));
                m_activeContacts.clear();
            }
        }
    }

    void ROS2ContactSensorComponent::AddNewContact(const AzPhysics::CollisionEvent& event)
    {
        AZ::Entity* contactedEntity = nullptr;
        AZ::ComponentApplicationBus::BroadcastResult(
            contactedEntity, &AZ::ComponentApplicationRequests::FindEntity, event.m_body2->GetEntityId());
        gazebo_msgs::msg::ContactState state;
        AZ_Assert(contactedEntity, "Invalid entity pointer value");
        state.collision1_name = ("ID: " + m_entityId.ToString() + " Name:" + m_entityName).c_str();
        state.collision2_name = ("ID: " + event.m_body2->GetEntityId().ToString() + " Name:" + contactedEntity->GetName()).c_str();
        geometry_msgs::msg::Wrench totalWrench;
        for (auto& contact : event.m_contacts)
        {
            if (contact.m_separation < ContactMaximumSeparation)
            {
                state.contact_positions.emplace_back(ROS2Conversions::ToROS2Vector3(contact.m_position));
                state.contact_normals.emplace_back(ROS2Conversions::ToROS2Vector3(contact.m_normal));

                geometry_msgs::msg::Wrench contactWrench;
                contactWrench.force = ROS2Conversions::ToROS2Vector3(contact.m_impulse);
                state.wrenches.push_back(AZStd::move(contactWrench));

                totalWrench.force.x += contact.m_impulse.GetX();
                totalWrench.force.y += contact.m_impulse.GetY();
                totalWrench.force.z += contact.m_impulse.GetZ();

                state.depths.emplace_back(contact.m_separation);
            }
        }

        state.total_wrench = AZStd::move(totalWrench);

        if (!state.contact_positions.empty())
        {
            AZStd::lock_guard<AZStd::mutex> lock(m_activeContactsMutex);
            m_activeContacts[event.m_body2->GetEntityId()] = AZStd::move(state);
        }
    }
} // namespace ROS2
