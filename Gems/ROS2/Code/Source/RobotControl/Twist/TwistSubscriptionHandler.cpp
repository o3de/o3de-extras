/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "TwistSubscriptionHandler.h"
#include <ROS2/RobotControl/Twist/TwistBus.h>
#include <ROS2/Utilities/ROS2Conversions.h>

namespace ROS2
{
    void TwistSubscriptionHandler::SendToBus(const geometry_msgs::msg::Twist& message)
    {
        const AZ::Vector3 linearVelocity = ROS2Conversions::FromROS2Vector3(message.linear);
        const AZ::Vector3 angularVelocity = ROS2Conversions::FromROS2Vector3(message.angular);
        TwistNotificationBus::Event(GetEntityId(), &TwistNotifications::TwistReceived, linearVelocity, angularVelocity);
    }
} // namespace ROS2
