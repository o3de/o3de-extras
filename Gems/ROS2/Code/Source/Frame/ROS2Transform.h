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
   /// Publishes transforms as standard ros2 tf2 messages. Static transforms are published once.
   // TODO - Rework this class (name, function). Separate broadcaster out of ROS2SystemComponent
   class ROS2Transform
   {
   public:
       ROS2Transform(AZStd::string parentFrame, AZStd::string childFrame, bool isDynamic);
       void Publish(const AZ::Transform& transform);

   private:
       geometry_msgs::msg::TransformStamped CreateTransformMessage(const AZ::Transform& transform);

       const AZStd::string m_parentFrame;
       const AZStd::string m_childFrame;
       bool m_isPublished = false;
       bool m_isDynamic;
   };
}  // namespace ROS2