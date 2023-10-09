/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Math/Quaternion.h>
#include <AzCore/Math/Transform.h>
#include <AzCore/Math/Vector3.h>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>

namespace ROS2
{
    //! Utility class for conversions between ROS2 types and O3DE (AZ::) types.
    namespace ROS2Conversions
    {
        AZ::Vector3 FromROS2Vector3(const geometry_msgs::msg::Vector3& ros2vector);
        geometry_msgs::msg::Vector3 ToROS2Vector3(const AZ::Vector3& azvector);
        geometry_msgs::msg::Point ToROS2Point(const AZ::Vector3& azvector);
        geometry_msgs::msg::Quaternion ToROS2Quaternion(const AZ::Quaternion& azquaternion);
        geometry_msgs::msg::Pose ToROS2Pose(const AZ::Transform& aztransform);
        AZ::Transform FromROS2Pose(const geometry_msgs::msg::Pose& ros2pose);
        AZ::Vector3 FromROS2Point(const geometry_msgs::msg::Point& ros2point);
        AZ::Quaternion FromROS2Quaternion(const geometry_msgs::msg::Quaternion& ros2quaternion);
        std::array<double, 9> ToROS2Covariance(const AZ::Matrix3x3& covariance);
    }; // namespace ROS2Conversions
} // namespace ROS2
