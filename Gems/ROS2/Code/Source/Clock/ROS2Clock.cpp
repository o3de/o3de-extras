/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <ROS2/Clock/ROS2Clock.h>
#include <ROS2/Clock/SimulationTimeSource.h>
#include <ROS2/ROS2Bus.h>
#include <rclcpp/qos.hpp>

namespace ROS2
{
    ROS2Clock::ROS2Clock()
        : m_timeSource(AZStd::make_unique<SimulationTimeSource>())
        , m_publishClock(true)
    {
    }

    ROS2Clock::ROS2Clock(AZStd::unique_ptr<ITimeSource> timeSource, bool publishClock)
        : m_timeSource(AZStd::move(timeSource))
        , m_publishClock(publishClock)
    {
    }

    void ROS2Clock::Activate()
    {
        m_timeSource->Activate();
    }

    void ROS2Clock::Deactivate()
    {
        m_timeSource->Deactivate();
    }

    builtin_interfaces::msg::Time ROS2Clock::GetROSTimestamp() const
    {
        return m_timeSource->GetROSTimestamp();
    }

    void ROS2Clock::Tick()
    {
        if (!m_publishClock)
        {
            return;
        }
        if (!m_clockPublisher)
        { // Lazy construct
            auto ros2Node = ROS2Interface::Get()->GetNode();

            rclcpp::ClockQoS qos;
            m_clockPublisher = ros2Node->create_publisher<rosgraph_msgs::msg::Clock>("/clock", qos);
        }

        rosgraph_msgs::msg::Clock msg;
        msg.clock = GetROSTimestamp();
        m_clockPublisher->publish(msg);
    }
} // namespace ROS2
