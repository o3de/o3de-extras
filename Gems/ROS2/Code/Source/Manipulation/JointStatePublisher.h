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
#include <ROS2/Manipulation/JointInfo.h>
#include <Utilities/PhysicsCallbackHandler.h>
#include <rclcpp/publisher.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

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
    class JointStatePublisher : public ROS2::Utils::PhysicsCallbackHandler
    {
    public:
        JointStatePublisher(const PublisherConfiguration& configuration, const JointStatePublisherContext& context);
        virtual ~JointStatePublisher() = default;

        void InitializePublisher(AZ::EntityId entityId);

    private:
        void PublishMessage();

        PublisherConfiguration m_configuration;
        JointStatePublisherContext m_context;

        std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::JointState>> m_jointStatePublisher;
        sensor_msgs::msg::JointState m_jointStateMsg;
        float m_timeElapsedSinceLastTick = 0.0f;

        AZStd::vector<AZStd::string> m_jointNames;
        AZStd::vector<JointInfo> m_jointInfos;

        // ROS2::Utils::PhysicsCallbackHandler overrides ...
        void OnPhysicsSimulationFinished(AzPhysics::SceneHandle sceneHandle, float deltaTime) override;
    };
} // namespace ROS2
