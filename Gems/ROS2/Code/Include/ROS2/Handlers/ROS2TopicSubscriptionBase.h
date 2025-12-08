/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include "HandlersRegistryUtils.h"
#include "IROS2HandlerBase.h"
#include <ROS2/Communication/TopicConfiguration.h>
#include <rclcpp/node.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>

namespace ROS2
{
    //! Base for each ROS 2 topic subscription handler with default QoS set to (RELIABLE/VOLATILE)
    template<typename RosMessageType>
    class ROS2TopicSubscriptionBase : public virtual IROS2HandlerBase
    {
    public:
        using SubscriptionHandle = std::shared_ptr<rclcpp::Subscription<RosMessageType>>;
        void Initialize(rclcpp::Node::SharedPtr& node) override
        {
            CreateSubscription(node);
        }

        ROS2TopicSubscriptionBase()
        {
            // if no topic configuration was passed, set default type from the class
            m_topicConfig.m_topic = GetDefaultName();
        }

        // inherit given configuration
        ROS2TopicSubscriptionBase(const ROS2::TopicConfiguration& topicConfig)
            : m_topicConfig(topicConfig)
        {
        }

        virtual ~ROS2TopicSubscriptionBase() = default;

        bool IsValid() const override
        {
            return m_subscriber != nullptr;
        }

    protected:
        // this method is called when message is received on topic
        virtual void HandleMessage(const RosMessageType& message) = 0;

    private:
        void CreateSubscription(rclcpp::Node::SharedPtr& node)
        {
            // get the topic name from the type name
            // passing an empty string to settings registry disables ROS 2 action
            AZStd::optional<AZStd::string> topicName = HandlersRegistryUtils::GetName(GetTypeName());

            // do not create a ROS 2 topic if the value from setreg is empty
            if (topicName.has_value())
            {
                if (topicName.value().empty())
                {
                    AZ_Trace(
                        "ROS2 Gem",
                        "Topic name for type %s is set to empty string, topic subscriber won't be created",
                        GetTypeName().data());
                    return;
                }

                m_topicConfig.m_topic = topicName.value();
            }

            m_subscriber = node->create_subscription<RosMessageType>(
                m_topicConfig.m_topic,
                m_topicConfig.GetQoS(),
                [this](const typename RosMessageType::SharedPtr message_ptr)
                {
                    HandleMessage(*message_ptr);
                });
        }
        ROS2::TopicConfiguration m_topicConfig;
        SubscriptionHandle m_subscriber;
    };

} // namespace ROS2
