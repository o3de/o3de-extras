/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "TwistSubscriptionHandler.h"
#include <ROS2/Utilities/ROS2Conversions.h>
#include <ROS2Controllers/RobotControl/Twist/TwistBus.h>

namespace ROS2Controllers
{
    void TwistSubscriptionHandler::SendToBus(const geometry_msgs::msg::Twist& message)
    {
        const AZ::Vector3 linearVelocity = ROS2::ROS2Conversions::FromROS2Vector3(message.linear);
        const AZ::Vector3 angularVelocity = ROS2::ROS2Conversions::FromROS2Vector3(message.angular);
        TwistNotificationBus::Event(GetEntityId(), &TwistNotifications::TwistReceived, linearVelocity, angularVelocity);
    }
} // namespace ROS2Controllers
