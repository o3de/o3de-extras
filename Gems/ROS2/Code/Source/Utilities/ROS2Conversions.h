/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Math/Vector3.h>
#include <AzCore/Math/Quaternion.h>
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/pose.hpp"

namespace ROS2
{
    class ROS2Conversions
    {
    public:
        static AZ::Vector3 FromROS2Vector3(const geometry_msgs::msg::Vector3& ros2vector);
        static geometry_msgs::msg::Vector3 ToROS2Vector3(const AZ::Vector3& azvector);
        static geometry_msgs::msg::Quaternion ToROS2Quaternion(const AZ::Quaternion& azquaternion);
    };
}  // namespace ROS2