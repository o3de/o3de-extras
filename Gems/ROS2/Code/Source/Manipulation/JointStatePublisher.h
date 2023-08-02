/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/Component/EntityId.h>
#include <ROS2/Communication/PublisherConfiguration.h>
#include <rclcpp/publisher.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <ROS2/Communication/FlexiblePublisher.h>

namespace ROS2
{
    struct JointStatePublisherContext
    {
        AZ::EntityId m_entityId;
        AZStd::string m_frameId;
        AZStd::string m_publisherNamespace;
    };

    //! A class responsible for publishing the joint positions on ROS2 /joint_states topic.
    //!< @see <a href="https://docs.ros2.org/latest/api/sensor_msgs/msg/JointState.html">jointState message</a>.
    class JointStatePublisher
    {
    public:
        JointStatePublisher(const PublisherConfiguration& configuration, const JointStatePublisherContext& context);

        //! Update time tick. This will result in state publishing if timing matches frequency.
        void OnTick(float deltaTime);

    private:
        void PublishMessage();

        PublisherConfiguration m_configuration;
        JointStatePublisherContext m_context;

        std::shared_ptr<FlexiblePublisher<sensor_msgs::msg::JointState>> m_jointStatePublisher;
        sensor_msgs::msg::JointState m_jointStateMsg;
        float m_timeElapsedSinceLastTick = 0.0f;
    };
} // namespace ROS2
