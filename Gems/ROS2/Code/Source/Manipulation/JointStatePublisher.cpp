/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <ROS2/Manipulation/JointsManipulationRequests.h>
#include <ROS2/ROS2Bus.h>
#include <ROS2/Utilities/ROS2Names.h>

#include "JointStatePublisher.h"
#include "ManipulationUtils.h"

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

    JointStatePublisher::~JointStatePublisher()
    {
        m_eventSourceAdapter.Stop();
        m_adaptedEventHandler.Disconnect();
    }

    void JointStatePublisher::PublishMessage()
    {
        std_msgs::msg::Header rosHeader;
        rosHeader.frame_id = ROS2Names::GetNamespacedName(m_context.m_publisherNamespace, m_context.m_frameId).data();
        rosHeader.stamp = ROS2::ROS2Interface::Get()->GetROSTimestamp();
        m_jointStateMsg.header = rosHeader;

        AZ_Assert(m_jointNames.size() == m_jointStateMsg.name.size(), "The expected message size doesn't match with the joint list size");

        for (size_t i = 0; i < m_jointStateMsg.name.size(); i++)
        {
            m_jointStateMsg.name[i] = m_jointNames[i].c_str();
            JointInfo& jointInfo = m_jointInfos[i];

            auto jointStateData = Utils::GetJointState(jointInfo);

            m_jointStateMsg.position[i] = jointStateData.position;
            m_jointStateMsg.velocity[i] = jointStateData.velocity;
            m_jointStateMsg.effort[i] = jointStateData.effort;
        }
        m_jointStatePublisher->publish(m_jointStateMsg);
    }

    void JointStatePublisher::InitializePublisher()
    {
        ManipulationJoints manipulatorJoints;
        JointsManipulationRequestBus::EventResult(manipulatorJoints, m_context.m_entityId, &JointsManipulationRequests::GetJoints);

        for (const auto& [jointName, jointInfo] : manipulatorJoints)
        {
            m_jointNames.push_back(jointName);
            m_jointInfos.push_back(jointInfo);
        }

        m_jointStateMsg.name.resize(manipulatorJoints.size());
        m_jointStateMsg.position.resize(manipulatorJoints.size());
        m_jointStateMsg.velocity.resize(manipulatorJoints.size());
        m_jointStateMsg.effort.resize(manipulatorJoints.size());

        m_eventSourceAdapter.SetFrequency(m_configuration.m_frequency);
        m_adaptedEventHandler = decltype(m_adaptedEventHandler)(
            [this](auto&&... args)
            {
                if (!m_configuration.m_publish)
                {
                    return;
                }
                PublishMessage();
            });
        m_eventSourceAdapter.ConnectToAdaptedEvent(m_adaptedEventHandler);
        m_eventSourceAdapter.Start();
    }
} // namespace ROS2
