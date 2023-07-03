/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <AzCore/Time/ITime.h>
#include <AzCore/std/algorithm.h>
#include <AzCore/std/containers/deque.h>
#include <AzCore/std/containers/vector.h>
#include <AzCore/std/sort.h>
#include <ROS2/Clock/SimulationClock.h>
#include <ROS2/ROS2Bus.h>
#include <rclcpp/qos.hpp>

namespace ROS2
{
    builtin_interfaces::msg::Time SimulationClock::GetROSTimestamp() const
    {
        const auto elapsedTime = GetElapsedTimeMicroseconds();

        builtin_interfaces::msg::Time timeStamp;
        timeStamp.sec = static_cast<int32_t>(elapsedTime / 1000000);
        timeStamp.nanosec = static_cast<uint32_t>((elapsedTime % 1000000) * 1000);
        return timeStamp;
    }

    int64_t SimulationClock::GetElapsedTimeMicroseconds() const
    {
        if (auto* timeSystem = AZ::Interface<AZ::ITime>::Get())
        {
            return static_cast<int64_t>(timeSystem->GetElapsedTimeUs());
        }
        else
        {
            AZ_Error("SimulationClock", false, "No ITime interface available for ROS2 Gem simulation clock");
            return 0;
        }
    }

    AZStd::chrono::duration<float, AZStd::chrono::seconds::period> SimulationClock::GetExpectedSimulationLoopTime() const
    {
        return AZStd::chrono::duration<AZ::s64, AZStd::chrono::microseconds::period>(m_currentMedian);
    }

    void SimulationClock::Tick()
    {
        auto elapsed = GetElapsedTimeMicroseconds();
        if (!m_clockPublisher)
        { // Lazy construct
            auto ros2Node = ROS2Interface::Get()->GetNode();

            rclcpp::ClockQoS qos;
            m_clockPublisher = ros2Node->create_publisher<rosgraph_msgs::msg::Clock>("/clock", qos);
        }

        rosgraph_msgs::msg::Clock msg;
        msg.clock = GetROSTimestamp();
        m_clockPublisher->publish(msg);

        AZ::s64 deltaTime = elapsed - m_lastExecutionTime;
        m_lastExecutionTime = elapsed;

        // statistics on execution time
        m_frameTimes.push_back(deltaTime);
        if (m_frameTimes.size() > FramesNumberForStats)
        {
            m_frameTimes.pop_front();
        }
        AZStd::vector<AZ::s64> frameTimeSorted{ m_frameTimes.begin(), m_frameTimes.end() };
        AZStd::sort(frameTimeSorted.begin(), frameTimeSorted.end());
        m_currentMedian = frameTimeSorted[frameTimeSorted.size() / 2];
    }
} // namespace ROS2
