/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <ROS2/Frame/ROS2Transform.h>
#include <ROS2/ROS2Bus.h>
#include <ROS2/Utilities/ROS2Conversions.h>
#include <ROS2/Utilities/ROS2Names.h>
#include <tf2_ros/qos.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace ROS2
{
    ROS2Transform::ROS2Transform(AZStd::string parentFrame, AZStd::string childFrame, bool isDynamic)
        : m_parentFrame(AZStd::move(parentFrame))
        , m_childFrame(AZStd::move(childFrame))
        , m_isDynamic(isDynamic)
    {
        auto splitName = ROS2Names::SplitIntoNamespaceAndName(childFrame);
        if (splitName.first.empty())
        {   // No namespace in child frame, no need for a publisher.
            return;
        }

        auto ros2Node = ROS2Interface::Get()->GetNode();
        rclcpp::QoS qos = tf2_ros::StaticBroadcasterQoS();
        if (m_isDynamic)
        {
            qos = tf2_ros::DynamicBroadcasterQoS();
        }
        AZStd::string fullTopic = ROS2Names::GetNamespacedName(splitName.first, splitName.second);
        m_transformPublisher = ros2Node->create_publisher<geometry_msgs::msg::TransformStamped>(fullTopic.data(), qos);
    }

    geometry_msgs::msg::TransformStamped ROS2Transform::CreateTransformMessage(const AZ::Transform& o3deTransform)
    {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = ROS2Interface::Get()->GetROSTimestamp();
        t.header.frame_id = m_parentFrame.data();
        t.child_frame_id = m_childFrame.data();
        t.transform.translation = ROS2Conversions::ToROS2Vector3(o3deTransform.GetTranslation());
        t.transform.rotation = ROS2Conversions::ToROS2Quaternion(o3deTransform.GetRotation());
        return t;
    }

    void ROS2Transform::Publish(const AZ::Transform& transform)
    {
        if (m_isPublished && !m_isDynamic)
        { // Only publish static transforms once
            return;
        }
        auto transformMessage = CreateTransformMessage(transform);
        ROS2Interface::Get()->BroadcastTransform(transformMessage, m_isDynamic);

        if (m_transformPublisher)
        {
            AZStd::string parent_id(transformMessage.header.frame_id.c_str());
            AZStd::string child_frame_id(transformMessage.child_frame_id.c_str());
            ROS2Names::StripNamespace(parent_id);
            ROS2Names::StripNamespace(child_frame_id);
            transformMessage.header.frame_id = parent_id.c_str();
            transformMessage.child_frame_id = child_frame_id.c_str();
            m_transformPublisher->publish(transformMessage);
        }

        m_isPublished = true;
    }
} // namespace ROS2
