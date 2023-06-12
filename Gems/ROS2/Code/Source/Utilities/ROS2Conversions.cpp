/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <AzCore/Math/Matrix3x3.h>
#include <AzCore/Math/Transform.h>
#include <ROS2/Utilities/ROS2Conversions.h>

namespace ROS2
{
    AZ::Vector3 ROS2Conversions::FromROS2Vector3(const geometry_msgs::msg::Vector3& ros2vector)
    { // Same coordinate systems - just translate types
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

    geometry_msgs::msg::Point ROS2Conversions::ToROS2Point(const AZ::Vector3& azvector)
    {
        geometry_msgs::msg::Point ros2point;
        ros2point.x = azvector.GetX();
        ros2point.y = azvector.GetY();
        ros2point.z = azvector.GetZ();
        return ros2point;
    }

    geometry_msgs::msg::Quaternion ROS2Conversions::ToROS2Quaternion(const AZ::Quaternion& azquaternion)
    {
        geometry_msgs::msg::Quaternion ros2Quaternion;
        ros2Quaternion.x = azquaternion.GetX();
        ros2Quaternion.y = azquaternion.GetY();
        ros2Quaternion.z = azquaternion.GetZ();
        ros2Quaternion.w = azquaternion.GetW();
        return ros2Quaternion;
    }

    geometry_msgs::msg::Pose ROS2Conversions::ToROS2Pose(const AZ::Transform& aztransform)
    {
        geometry_msgs::msg::Pose pose;
        pose.position = ToROS2Point(aztransform.GetTranslation());
        pose.orientation = ToROS2Quaternion(aztransform.GetRotation());
        return pose;
    }

    AZ::Transform ROS2Conversions::FromROS2Pose(const geometry_msgs::msg::Pose& ros2pose)
    {
        return { FromROS2Point(ros2pose.position), FromROS2Quaternion(ros2pose.orientation), 1.0f };
    }

    AZ::Vector3 ROS2Conversions::FromROS2Point(const geometry_msgs::msg::Point& ros2point)
    {
        return AZ::Vector3(ros2point.x, ros2point.y, ros2point.z);
    }

    AZ::Quaternion ROS2Conversions::FromROS2Quaternion(const geometry_msgs::msg::Quaternion& ros2quaternion)
    {
        AZ::Quaternion azquaternion;
        azquaternion.SetX(ros2quaternion.x);
        azquaternion.SetY(ros2quaternion.y);
        azquaternion.SetZ(ros2quaternion.z);
        azquaternion.SetW(ros2quaternion.w);
        return azquaternion;
    }

    std::array<double, 9> ROS2Conversions::ToROS2Covariance(const AZ::Matrix3x3& covariance)
    {
        std::array<double, 9> ros2Covariance;
        for (int i = 0; i < 9; ++i)
        {
            ros2Covariance[i] = covariance.GetElement(i / 3, i % 3);
        }
        return ros2Covariance;
    }

} // namespace ROS2
