/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <ROS2/Communication/SubscriptionHandler.h>
#include <ackermann_msgs/msg/ackermann_drive.hpp>

namespace ROS2
{
    class AckermannSubscriptionHandler : public SubscriptionHandler<ackermann_msgs::msg::AckermannDrive>
    {
    private:
        void ExecuteUponMessage(const ackermann_msgs::msg::AckermannDrive& message) override;
    };
} // namespace ROS2
