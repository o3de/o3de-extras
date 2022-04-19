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

namespace ROS2
{
   /// Publishes transforms as standard ros2 tf2 messages. Static transforms are published once.
   // TODO - differentiate between static and dynamic publisher. Rework this class (name, function)
   class TransformPublisher
   {
   public:
       TransformPublisher(const AZStd::string& parentFrame, const AZStd::string &childFrame,
                          const AZ::Transform& transform);
   };
}  // namespace ROS2