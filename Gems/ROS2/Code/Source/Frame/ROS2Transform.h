/*
* Copyright (c) Contributors to the Open 3D Engine Project.
* For complete copyright and license terms please see the LICENSE at the root of this distribution.
*
* SPDX-License-Identifier: Apache-2.0 OR MIT
*
*/
#pragma once

#include <AzCore/Math/Transform.h>
#include <AzCore/std/string/string.h>
#include <AzCore/std/smart_ptr/unique_ptr.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace ROS2
{
    namespace Internal
    {
        class TransformPublisher
        {
        public:
            TransformPublisher() = default;
            virtual ~TransformPublisher() = default;
            static AZStd::unique_ptr<TransformPublisher> CreateTransformPublisher(bool isDynamic);
            virtual void Publish(const geometry_msgs::msg::TransformStamped& transformMessage) = 0;
        };
    }

   /// Publishes transforms as standard ros2 tf2 messages. Static transforms are published once.
   // TODO - differentiate between static and dynamic publisher. Rework this class (name, function)
   class ROS2Transform
   {
   public:
       ROS2Transform(AZStd::string parentFrame, AZStd::string childFrame, bool isDynamic);
       void Publish(const AZ::Transform& transform);

   private:
       geometry_msgs::msg::TransformStamped CreateTransformMessage(const AZ::Transform& transform);

       const AZStd::string m_parentFrame;
       const AZStd::string m_childFrame;

       AZStd::unique_ptr<Internal::TransformPublisher> m_transformPublisher;
   };
}  // namespace ROS2