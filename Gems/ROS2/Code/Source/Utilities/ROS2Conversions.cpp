/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#include "ROS2Conversions.h"

namespace ROS2
{   // Same coordinate systems - just translate types
    AZ::Vector3 ROS2Conversions::FromROS2Vector3(const geometry_msgs::msg::Vector3& ros2vector)
    {
        return AZ::Vector3(ros2vector.x, ros2vector.y, ros2vector.z);
    }

    geometry_msgs::msg::Vector3 ROS2Conversions::ToROS2Vector3(const AZ::Vector3& azvector)
    {
        geometry_msgs::msg::Vector3 ros2vector;
        ros2vector.x = azvector.GetX();
        ros2vector.y = azvector.GetY();
        ros2vector.z = azvector.GetZ();
        return ros2vector;
    }
}  // namespace ROS2