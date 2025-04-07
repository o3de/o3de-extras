/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include "ITimeSource.h"
#include <AzCore/std/chrono/chrono.h>
#include <AzCore/std/smart_ptr/unique_ptr.h>
#include <rclcpp/publisher.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <AzCore/Outcome/Outcome.h>
#include <AzCore/std/string/string.h>

namespace ROS2
{
    //! The ROS2Clock provides ROS timestamps as builtin_interface::msg::Time messages.
    //! The clock can use different types of the time sources and publish the current
    //! time to the ROS 2 `/clock/` topic. The published time can be used with
    //! the /use_sim_time parameter set to true.
    class ROS2Clock
    {

    public:
        ROS2Clock();
        ROS2Clock(AZStd::unique_ptr<ITimeSource> timeSource, bool publishClock);
        ~ROS2Clock() = default;

        void Activate();
        void Deactivate();

        builtin_interfaces::msg::Time GetROSTimestamp() const;

        AZ::Outcome<void, AZStd::string> AdjustTime(const builtin_interfaces::msg::Time & time) const;

        //! Update time in the ROS 2 ecosystem.
        //! This will publish current time to the ROS 2 `/clock` topic, if Clock is configured to do it.
        void Tick();

    private:
        rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr m_clockPublisher;
        AZStd::unique_ptr<ITimeSource> m_timeSource;
        bool m_publishClock;
    };
} // namespace ROS2
