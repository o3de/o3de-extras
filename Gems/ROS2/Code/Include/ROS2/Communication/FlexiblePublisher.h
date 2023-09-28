/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <ROS2/Communication/PublisherConfiguration.h>
#include <ROS2/Utilities/ROS2Names.h>
#include <ROS2/ROS2Bus.h>
#include <rclcpp/create_publisher.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>

namespace ROS2
{

    /* In ROS rolling, publishers now have virtual publish methods, allowing for a more elegant design:
     * However, we need to support Humble.
     * //! A wrapper class to allow custom behavior when or instead of publishing, for example, writing to ros 2 bags directly.
     * //! To create, call node->create_publisher as usual, but make sure to set the third template argument (PublisherT) to this class.
    template<typename MessageT, typename AllocatorT = std::allocator<void>>
    class FlexiblePublisher : public rclcpp::Publisher<MessageT, AllocatorT>
    {
    public:
        void publish(typename rclcpp::Publisher<MessageT, AllocatorT>::MessageUniquePtr msg) override
        {
            rclcpp::Publisher<MessageT, AllocatorT>::template publish(msg);
        }

        FlexiblePublisher(
            rclcpp::node_interfaces::NodeBaseInterface* node_base,
            const std::string& topic,
            const rclcpp::QoS& qos,
            const rclcpp::PublisherOptionsWithAllocator<AllocatorT>& options)
            : rclcpp::Publisher<MessageT, AllocatorT>(node_base, topic, qos, options)
        {
        }
    };
    */

    template<typename MessageT>
    struct FlexiblePublishBehavior
    {
        using MessageProcessingRefFn = std::function<void(const MessageT& msgs)>;
        MessageProcessingRefFn m_prePublish = []([[maybe_unused]] const MessageT& msgs)
        {
        };
        MessageProcessingRefFn m_postPublish = []([[maybe_unused]] const MessageT& msgs)
        {
        };
        bool m_skipPublishing = false;
    };

    //! Helper class to include custom handling of message publishing, including writing to ros 2 bags directly.
    template<typename MessageT>
    class FlexiblePublisher
    {
    public:
        static FlexiblePublishBehavior<MessageT> GetLogBehavior()
        {
            FlexiblePublishBehavior<MessageT> behavior;
            behavior.m_prePublish = [](const MessageT& msg) { AZ_Warning("FlexiblePublisher", false, "Publishing\n"); };
            return behavior;
        }

        FlexiblePublisher(const TopicConfiguration& topicConfiguration, const AZStd::string& rosNamespace)
        {
            m_flexibleBehavior = GetLogBehavior(); // Get behavior from anywhere, for example a system component
            AZStd::string topic = ROS2Names::GetNamespacedName(rosNamespace, topicConfiguration.m_topic);
            auto ros2Node = ROS2Interface::Get()->GetNode();
            m_publisher = ros2Node->create_publisher<MessageT>(topic.data(), topicConfiguration.GetQoS());
        }

        void publish(const MessageT& msg)
        {
            m_flexibleBehavior.m_prePublish(msg);
            if (m_flexibleBehavior.m_skipPublishing)
            {
                return;
            }
            m_publisher->publish(msg);
            m_flexibleBehavior.m_postPublish(msg);
        }

    private:
        FlexiblePublishBehavior<MessageT> m_flexibleBehavior;
        std::shared_ptr<rclcpp::Publisher<MessageT>> m_publisher;
    };
} // namespace ROS2
