/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/Component/Component.h>
#include <AzCore/Component/ComponentApplication.h>
#include <AzCore/Component/ComponentApplicationBus.h>
#include <AzCore/Component/Entity.h>
#include <AzCore/Component/EntityBus.h>
#include <AzCore/Component/EntityId.h>
#include <AzCore/Debug/Trace.h>
#include <AzCore/std/string/string.h>
#include <AzFramework/Entity/EntityContextBus.h>
#include <ROS2/Communication/PublisherConfiguration.h>
#include <ROS2/ROS2Bus.h>
#include <ROS2/Sensor/ROS2SensorComponent.h>
#include <ROS2/Utilities/ROS2Names.h>
#include <exception>
#include <memory>
#include <rclcpp/create_publisher.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>

namespace ROS2
{
    //! Helper class to include custom handling of message publishing, including writing to ros 2 bags directly.
    //! Currently used for handling ros 2 exceptions.
    template<typename MessageT>
    class FlexiblePublisher : AZ::EntityBus::Handler
    {
    public:
        FlexiblePublisher(
            const TopicConfiguration& topicConfiguration,
            const AZStd::string& rosNamespace,
            const AZ::EntityId entityId,
            const AZStd::string& componentName)
            : m_componentsName(componentName)
            , m_entityId(entityId)
        {
            AZStd::string topic = ROS2Names::GetNamespacedName(rosNamespace, topicConfiguration.m_topic);
            AZ::EntityBus::Handler::BusConnect(entityId);
            auto ros2Node = ROS2Interface::Get()->GetNode();

            try
            {
                m_publisher = ros2Node->create_publisher<MessageT>(topic.data(), topicConfiguration.GetQoS());
            }
            catch (std::exception& e)
            {
                AZ_Error("FlexiblePublisher", false, "%s: %s", m_componentsName.c_str(), e.what());
                m_isError = true;
            }
        }

        ~FlexiblePublisher()
        {
            AZ::EntityBus::Handler::BusDisconnect();
        }

        void OnEntityActivated(const AZ::EntityId& entityId) override
        {
            if (m_isError)
            {
                AZ::Entity* entity = nullptr;
                AZ::ComponentApplicationBus::BroadcastResult(entity, &AZ::ComponentApplicationRequests::FindEntity, entityId);
                AZ_Assert(entity, "FlexiblePublisher: %s Entity with given id doesn't exist.", m_componentsName.c_str());
                entity->Deactivate();
            }
        }

        void publish(const MessageT& msg)
        {
            try
            {
                m_publisher->publish(msg);
            }
            catch (std::exception& e)
            {
                AZ_Error("FlexiblePublisher", false, "%s: %s", m_componentsName.c_str(), e.what());
                AZ::Entity* entity = nullptr;
                AZ::ComponentApplicationBus::BroadcastResult(entity, &AZ::ComponentApplicationRequests::FindEntity, m_entityId);
                AZ_Assert(entity, "FlexiblePublisher: %s Entity with given id doesn't exist.", m_componentsName.c_str());
                entity->Deactivate();
            }
        }

    private:
        std::shared_ptr<rclcpp::Publisher<MessageT>> m_publisher;
        AZ::EntityId m_entityId;
        AZStd::string m_componentsName;
        bool m_isError = false;
    };
} // namespace ROS2
