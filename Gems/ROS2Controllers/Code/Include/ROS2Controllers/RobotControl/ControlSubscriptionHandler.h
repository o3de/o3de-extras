/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <ROS2/Communication/TopicConfiguration.h>
#include <ROS2/Frame/ROS2FrameComponent.h>
#include <ROS2/ROS2Bus.h>
#include <ROS2/Utilities/ROS2Names.h>
#include <rclcpp/rclcpp.hpp>

namespace ROS2
{
    //! Component extension enabling polymorphic use of generics.
    class IControlSubscriptionHandler
    {
    public:
        //! Interface handling component activation
        //! Only activated IComponentActivationHandler will receive and process control messages.
        //! @param entity Activation context for the owning Component - the entity it belongs to.
        //! @param subscriberConfiguration configuration with topic and qos
        virtual void Activate(const AZ::Entity* entity, const TopicConfiguration& subscriberConfiguration) = 0;
        //! Interface handling component deactivation
        virtual void Deactivate() = 0;
        virtual ~IControlSubscriptionHandler() = default;
    };

    //! The generic class for handling subscriptions to ROS2 control messages of different types.
    //! @see ControlConfiguration::Steering.
    template<typename T>
    class ControlSubscriptionHandler : public IControlSubscriptionHandler
    {
    public:
        void Activate(const AZ::Entity* entity, const TopicConfiguration& subscriberConfiguration) override final
        {
            m_active = true;
            m_entityId = entity->GetId();
            if (!m_controlSubscription)
            {
                auto ros2Frame = entity->FindComponent<ROS2FrameComponent>();
                AZStd::string namespacedTopic = ROS2Names::GetNamespacedName(ros2Frame->GetNamespace(), subscriberConfiguration.m_topic);

                auto ros2Node = ROS2Interface::Get()->GetNode();
                m_controlSubscription = ros2Node->create_subscription<T>(
                    namespacedTopic.data(),
                    subscriberConfiguration.GetQoS(),
                    [this](const T& message)
                    {
                        OnControlMessage(message);
                    });
            }
        };

        void Deactivate() override final
        {
            m_active = false;
            m_controlSubscription.reset(); // Note: topic and qos can change, need to re-subscribe
        };

        virtual ~ControlSubscriptionHandler() = default;

    protected:
        AZ::EntityId GetEntityId() const
        {
            return m_entityId;
        }

    private:
        void OnControlMessage(const T& message)
        {
            if (!m_active)
            {
                return;
            }

            SendToBus(message);
        };

        virtual void SendToBus(const T& message) = 0;

        AZ::EntityId m_entityId;
        bool m_active = false;
        typename rclcpp::Subscription<T>::SharedPtr m_controlSubscription;
    };
} // namespace ROS2
