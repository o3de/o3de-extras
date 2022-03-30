/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include "RobotControl.h"
#include "geometry_msgs/msg/twist.hpp"

namespace ROS2
{
    class TwistControl : public RobotControl<geometry_msgs::msg::Twist>
    {
    private:
        void ApplyControl(const geometry_msgs::msg::Twist& message) override;
        void SetTargetComponent(const AZ::Entity* entity) override;

        AZ::EntityId m_entityID;
    };
}  // namespace ROS2