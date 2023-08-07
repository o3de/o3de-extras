/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "JointStatePublisher.h"
#include "ManipulationUtils.h"
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

    void JointStatePublisher::InitializePublisher(AZ::EntityId entityId)
    {
        ManipulationJoints manipulatorJoints;
        JointsManipulationRequestBus::EventResult(manipulatorJoints, m_context.m_entityId, &JointsManipulationRequests::GetJoints);

        for (auto& [jointName, jointInfo] : manipulatorJoints)
        {
            m_jointNames.push_back(jointName);
            m_jointInfos.push_back(jointInfo);
        }

        m_jointStateMsg.name.resize(manipulatorJoints.size());
        m_jointStateMsg.position.resize(manipulatorJoints.size());
        m_jointStateMsg.velocity.resize(manipulatorJoints.size());
        m_jointStateMsg.effort.resize(manipulatorJoints.size());

        InstallPhysicalCallback();
    }

    void JointStatePublisher::OnPhysicsSimulationFinished(AzPhysics::SceneHandle sceneHandle [[maybe_unused]], float deltaTime)
    {
        AZ_Assert(m_configuration.m_frequency > 0.f, "JointPublisher frequency must be greater than zero");
        auto frameTime = 1.f / m_configuration.m_frequency;

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
