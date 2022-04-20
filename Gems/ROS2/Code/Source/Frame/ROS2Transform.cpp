/*
* Copyright (c) Contributors to the Open 3D Engine Project.
* For complete copyright and license terms please see the LICENSE at the root of this distribution.
*
* SPDX-License-Identifier: Apache-2.0 OR MIT
*
*/

#include "Frame/ROS2Transform.h"
#include "ROS2/ROS2Bus.h"
#include "Utilities/ROS2Conversions.h"

namespace ROS2
{
    namespace Internal
    {
        class StaticTransformPublisher : public TransformPublisher
        {
        public:
            void Publish(const geometry_msgs::msg::TransformStamped& transformMessage) override
            {
                if (m_published)
                    return; // Consider adding a warning here as well

                return ROS2Interface::Get()->BroadcastStaticTransform(transformMessage);
                m_published = true;
            }

        private:
            bool m_published = false;
        };

        class DynamicTransformPublisher : public TransformPublisher
        {
        public:
            DynamicTransformPublisher()
            {
                auto ros2Node = ROS2Interface::Get()->GetNode();

                // TODO - determine whether default QoS is good and what is good practice for queue size. Compare to tf2 broadcaster
                const size_t keepLastQueueSize = 10;
                m_dynamicTFPublisher = ros2Node->create_publisher<geometry_msgs::msg::TransformStamped>("/tf", keepLastQueueSize);
            }

            void Publish(const geometry_msgs::msg::TransformStamped& transformMessage) override
            {
                m_dynamicTFPublisher->publish(transformMessage);
            }

        private:
            rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr m_dynamicTFPublisher;
        };

        AZStd::unique_ptr<TransformPublisher> TransformPublisher::CreateTransformPublisher(bool isDynamic)
        {
            if (isDynamic)
            {
                return std::make_unique<DynamicTransformPublisher>();
            }
            else
            {
                return std::make_unique<StaticTransformPublisher>();
            }
        }
    }

    ROS2Transform::ROS2Transform(const AZStd::string& parentFrame, const AZStd::string& childFrame, bool isDynamic)
        : m_parentFrame(parentFrame), m_childFrame(childFrame)
    {
        m_transformPublisher = Internal::TransformPublisher::CreateTransformPublisher(isDynamic);
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
        m_transformPublisher->Publish(CreateTransformMessage(transform));
    }
}  // namespace ROS2
