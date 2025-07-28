/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ROS2TimeSource.h"
#include <ROS2/ROS2Bus.h>

namespace ROS2
{
    builtin_interfaces::msg::Time ROS2TimeSource::GetROSTimestamp() const
    {
        auto ros2Node = ROS2Interface::Get()->GetNode();
            AZ_Assert(ros2Node, "No ros 2 node");
        builtin_interfaces::msg::Time timeStamp = ros2Node->get_clock()->now();
        return timeStamp;
    }

    AZ::Outcome<void, AZStd::string> ROS2TimeSource::AdjustTime(const builtin_interfaces::msg::Time& time)
    {
        return AZ::Failure(AZStd::string("ROS2TimeSource does not support setting a specific time."));
    }

} // namespace ROS2
