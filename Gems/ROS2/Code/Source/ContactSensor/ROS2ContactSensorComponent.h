/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/Component/EntityId.h>
#include <AzCore/RTTI/ReflectContext.h>
#include <AzCore/std/containers/unordered_map.h>
#include <AzCore/std/parallel/mutex.h>
#include <AzCore/std/string/string.h>
#include <AzFramework/Physics/Common/PhysicsSimulatedBodyEvents.h>
#include <ROS2/Sensor/Events/TickBasedSource.h>
#include <ROS2/Sensor/ROS2SensorComponentBase.h>
#include <gazebo_msgs/msg/contact_state.hpp>
#include <gazebo_msgs/msg/contacts_state.hpp>
#include <rclcpp/publisher.hpp>

namespace ROS2
{
    //! Contact sensor detects collisions between two objects.
    //! It reports the location of the contact associated forces.
    //! This component publishes a contact_sensor topic.
    //! It doesn't measure torque.
    class ROS2ContactSensorComponent : public ROS2SensorComponentBase<TickBasedSource>
    {
    public:
        AZ_COMPONENT(ROS2ContactSensorComponent, "{91272e66-c9f1-4aa2-a9d5-98eaa4ef4e9a}", SensorBaseType);
        ROS2ContactSensorComponent();
        ~ROS2ContactSensorComponent() = default;

        static void Reflect(AZ::ReflectContext* context);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        //////////////////////////////////////////////////////////////////////////
        // Component overrides
        void Activate() override;
        void Deactivate() override;
        //////////////////////////////////////////////////////////////////////////

    private:
        //////////////////////////////////////////////////////////////////////////
        void FrequencyTick();

        void AddNewContact(const AzPhysics::CollisionEvent& event);

        AZ::EntityId m_entityId;
        AZStd::string m_entityName = "";

        AzPhysics::SimulatedBodyEvents::OnCollisionBegin::Handler m_onCollisionBeginHandler;
        AzPhysics::SimulatedBodyEvents::OnCollisionPersist::Handler m_onCollisionPersistHandler;
        AzPhysics::SimulatedBodyEvents::OnCollisionEnd::Handler m_onCollisionEndHandler;

        std::shared_ptr<rclcpp::Publisher<gazebo_msgs::msg::ContactsState>> m_contactsPublisher;

        AZStd::unordered_map<AZ::EntityId, gazebo_msgs::msg::ContactState> m_activeContacts;
        AZStd::mutex m_activeContactsMutex;
    };
} // namespace ROS2
