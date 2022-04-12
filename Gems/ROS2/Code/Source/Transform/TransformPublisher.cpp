/*
* Copyright (c) Contributors to the Open 3D Engine Project.
* For complete copyright and license terms please see the LICENSE at the root of this distribution.
*
* SPDX-License-Identifier: Apache-2.0 OR MIT
*
*/

#include "Transform/TransformPublisher.h"
#include "ROS2/ROS2Bus.h"
#include "Utilities/ROS2Conversions.h"
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace ROS2
{
    TransformPublisher::TransformPublisher(
        const AZStd::string& parentFrame, const AZStd::string& childFrame, const AZ::Transform& o3deTransform)
    {
        auto ros2Node = ROS2Interface::Get()->GetNode();
        m_staticTFPublisher = AZStd::make_unique<tf2_ros::StaticTransformBroadcaster>(ros2Node);

        geometry_msgs::msg::TransformStamped t;

        t.header.stamp = ROS2Interface::Get()->GetROSTimestamp();
        t.header.frame_id = parentFrame.data();
        t.child_frame_id = childFrame.data();
        t.transform.translation = ROS2Conversions::ToROS2Vector3(o3deTransform.GetTranslation());
        t.transform.rotation = ROS2Conversions::ToROS2Quaternion(o3deTransform.GetRotation());
        m_staticTFPublisher->sendTransform(t);
    }
}  // namespace ROS2
