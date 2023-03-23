/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/std/chrono/chrono.h>
#include <AzCore/std/containers/deque.h>
#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/publisher.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

namespace ROS2
{
    //! Simulation clock which can tick and serve time stamps.
    class SimulationClock
    {
        static constexpr size_t FramesNumberForStats = 60;

    public:
        //! Get simulation time as ROS2 message.
        //! @see ROS2Requests::GetROSTimestamp() for more details.
        builtin_interfaces::msg::Time GetROSTimestamp() const;

        //! Update time in the ROS 2 ecosystem.
        //! This will publish current time to the ROS 2 `/clock` topic.
        void Tick();

        //! Returns an expected loop time of simulation. It is an estimation from past frames.
        AZStd::chrono::duration<float, AZStd::chrono::seconds::period> GetExpectedSimulationLoopTime() const;

    private:
        //! Get the time since start of sim, scaled with t_simulationTickScale
        int64_t GetElapsedTimeMicroseconds() const;

        AZ::s64 m_lastExecutionTime{ 0 };

        rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr m_clockPublisher;
        AZ::s64 m_currentMedian{ 0 };
        AZStd::deque<AZ::s64> m_frameTimes;
    };
} // namespace ROS2