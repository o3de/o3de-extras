/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <ROS2/Clock/ROS2TimeSource.h>
#include <ROS2/ROS2Bus.h>

namespace ROS2
{
    builtin_interfaces::msg::Time ROS2TimeSource::GetROSTimestamp() const
    {
        auto ros2Node = ROS2Interface::Get()->GetNode();
        builtin_interfaces::msg::Time timeStamp = ros2Node->get_clock()->now();
        return timeStamp;
    }
} // namespace ROS2
