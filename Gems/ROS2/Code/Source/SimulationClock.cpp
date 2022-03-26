#include "SimulationClock.h"
#include <rclcpp/qos.hpp>

using namespace ROS2;

builtin_interfaces::msg::Time SimulationClock::GetROSTimestamp() const
{
    auto elapsedTime = GetElapsedTimeMicroseconds();

    builtin_interfaces::msg::Time time_stamp;
    time_stamp.sec = static_cast<int32_t>(elapsedTime / 1000000);
    time_stamp.nanosec = static_cast<uint32_t>((elapsedTime % 1000000) * 1000);
    return time_stamp;
}

int64_t SimulationClock::GetElapsedTimeMicroseconds() const
{
    if (auto *timeSystem = AZ::Interface<AZ::ITime>::Get())
    {
        return static_cast<int64_t>(timeSystem->GetElapsedTimeUs());
    }
    else
    {
        AZ_Warning("ROS2SystemComponent", false, "No ITime interface available");
        return 0;
    }
}

void SimulationClock::Tick()
{
    if (!m_clockPublisher)
    {   //Lazy construct
        auto ros2_node = ROS2Interface::Get()->GetNode();

        // Standard QoS for /clock topic is best_effort, keep_last 1
        rclcpp::QoS qos(1);
        qos.best_effort();
        m_clockPublisher = ros2_node->create_publisher<rosgraph_msgs::msg::Clock>("/clock", qos);
    }

    rosgraph_msgs::msg::Clock msg;
    msg.clock = GetROSTimestamp();
    m_clockPublisher->publish(msg);
}