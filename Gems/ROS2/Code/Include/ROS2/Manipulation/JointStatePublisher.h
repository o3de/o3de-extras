#pragma once

#include <AzCore/Entity/EntityId.h>
#include <rclcpp/publisher.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace ROS2
{
    struct JointStatePublisherConfiguration
    {
        AZStd::string m_publisherNamespace;
        TopicConfiguration m_topicConfiguration;
        AZStd::string m_frameId;
        float m_frequency = 10;
    };

    //! A class responsible for publishing the joint positions on ROS2 /joint_states topic.
    //!< @see <a href="https://docs.ros2.org/latest/api/sensor_msgs/msg/JointState.html">jointState message</a>.
    class JointStatePublisher
    {
    public:
        JointStatePublisher(const JointStatePublisherConfiguration& configuration, const AZ::EntityId& entityId) = default;

        //! Update time tick. This will result in state publishing if timing matches frequency.
        void OnTick(float deltaTime)

    private:
        void PublishMessage();

        JointStatePublisherConfiguration m_configuration;
        AZ::EntityId m_entityId;
        std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::JointState>> m_jointStatePublisher;
        sensor_msgs::msg::JointState m_jointStateMsg;
        float m_timeElapsedSinceLastTick = 0.0f;
    };
} // namespace ROS2
