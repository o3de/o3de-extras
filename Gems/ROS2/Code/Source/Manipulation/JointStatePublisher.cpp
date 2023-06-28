/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "JointStatePublisher.h"
#include <ROS2/Manipulation/JointsManipulationRequests.h>
#include <ROS2/ROS2Bus.h>
#include <ROS2/Utilities/ROS2Names.h>

namespace ROS2
{
    JointStatePublisher::JointStatePublisher(const PublisherConfiguration& configuration, const JointStatePublisherContext& context)
        : m_configuration(configuration)
        , m_context(context)
    {
        auto topicConfiguration = m_configuration.m_topicConfiguration;
        AZStd::string topic = ROS2Names::GetNamespacedName(context.m_publisherNamespace, topicConfiguration.m_topic);
        auto ros2Node = ROS2Interface::Get()->GetNode();
        m_jointStatePublisher = ros2Node->create_publisher<sensor_msgs::msg::JointState>(topic.data(), topicConfiguration.GetQoS());
    }

    void JointStatePublisher::PublishMessage()
    {
        std_msgs::msg::Header rosHeader;
        rosHeader.frame_id = ROS2Names::GetNamespacedName(m_context.m_publisherNamespace, m_context.m_frameId).data();
        rosHeader.stamp = ROS2::ROS2Interface::Get()->GetROSTimestamp();
        m_jointStateMsg.header = rosHeader;

        ManipulationJoints manipulatorJoints;
        JointsManipulationRequestBus::EventResult(manipulatorJoints, m_context.m_entityId, &JointsManipulationRequests::GetJoints);

        m_jointStateMsg.name.resize(manipulatorJoints.size());
        m_jointStateMsg.position.resize(manipulatorJoints.size());
        m_jointStateMsg.velocity.resize(manipulatorJoints.size());
        m_jointStateMsg.effort.resize(manipulatorJoints.size());
        size_t i = 0;
        for (const auto& [jointName, jointInfo] : manipulatorJoints)
        {
            AZ::Outcome<float, AZStd::string> result;
            JointsManipulationRequestBus::EventResult(result, m_context.m_entityId, &JointsManipulationRequests::GetJointPosition, jointName);
            auto currentJointPosition = result.GetValue();

            m_jointStateMsg.name[i] = jointName.c_str();
            m_jointStateMsg.position[i] = currentJointPosition;
            // TODO - fill in velocity and effort
            m_jointStateMsg.velocity[i] = 0.0;
            m_jointStateMsg.effort[i] = 0.0;
            i++;
        }
        m_jointStatePublisher->publish(m_jointStateMsg);
    }

    void JointStatePublisher::OnTick(float deltaTime)
    { // TODO - refactor common publishing schemes for state publishers (consistent with sensors)
        AZ_Assert(m_configuration.m_frequency > 0, "JointPublisher frequency must be greater than zero");
        auto frameTime = 1 / m_configuration.m_frequency;

        m_timeElapsedSinceLastTick += deltaTime;
        if (m_timeElapsedSinceLastTick < frameTime)
        {
            return;
        }

        m_timeElapsedSinceLastTick -= frameTime;
        if (deltaTime > frameTime)
        { // Frequency higher than possible, not catching up, just keep going with each frame.
            m_timeElapsedSinceLastTick = 0.0f;
        }
        PublishMessage();
    }
} // namespace ROS2
