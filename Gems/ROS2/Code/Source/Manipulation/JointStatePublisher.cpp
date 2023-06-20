/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "JointPublisher.h"

namespace ROS2
{
    void JointStatePublisher::JointStatePublisher(const JointStatePublisherConfiguration& configuration, const AZ::EntityId& m_entityId)
        : mConfiguration(configuration)
        , m_entityId(entityId)
    {
        auto topicConfiguration = m_configuration.m_topicConfiguration;
        AZStd::string topic = ROS2Names::GetNamespacedName(configuration.m_publisherNamespace, topicConfiguration.m_topic);
        m_jointstatePublisher = ros2Node->create_publisher<sensor_msgs::msg::JointState>(topic.data(), topicConfiguration.GetQoS().GetQoS);
    }

    void JointStatePublisher::PublishMessage()
    {
        std_msgs::msg::Header rosHeader;
        rosHeader.frame_id = ROS2Names::GetNamespacedName(m_configuration.m_publisherNamespace, m_configuration.m_frameId).data();
        rosHeader.stamp = ROS2::ROS2Interface::Get()->GetROSTimestamp();
        m_jointStateMsg.header = rosHeader;

        ManipulatorRequestBus::ManipulatorJoints manipulatorJoints;
        ManipulatorBus::EventResult(manipulatorJoints, m_entityId, &ManipulatorRequests::GetManipulatorJoints);

        m_jointStateMsg.name.resize(manipulatorJoints.size());
        m_jointStateMsg.position.resize(manipulatorJoints.size());
        m_jointStateMsg.velocity.resize(manipulatorJoints.size());
        m_jointStateMsg.effort.resize(manipulatorJoints.size());
        size_t i = 0;
        for (const auto& [jointName, jointInfo] : manipulatorJoints)
        {
            AZ::Outcome<float, AZStd::string> result;
            ManipulatorBus::EventResult(result, m_entityID, &ManipulatorRequests::GetSingleDOFJointPosition, jointName);
            auto currentJointPosition = result.value();

            m_jointStateMsg.position[i] = currentJointPosition;
            // TODO - fill in velocity and effort
            m_jointStateMsg.name[i] = jointName.GetCStr();
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
            return;

        m_timeElapsedSinceLastTick -= frameTime;
        if (deltaTime > frameTime)
        { // Frequency higher than possible, not catching up, just keep going with each frame.
            m_timeElapsedSinceLastTick = 0.0f;
        }
        PublishMessage();
    }
} // namespace ROS2
