/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include "RobotControl/RobotControl.h"
#include <geometry_msgs/msg/twist.hpp>

namespace ROS2
{
    //! Specialization of RobotControl for Twist messages.
    //! Twist control moves robot through supplying linear and angular velocities.
    //! The control message can be broadcast to EBus system or directly applied to a selected body.
    //! This class is used through ROS2RobotControlComponent.
    //! @note How these velocities are reached might depend on a particular robot drive.
    //! @see <a href="https://docs.ros2.org/latest/api/geometry_msgs/msg/Twist.html">ROS2 Twist message</a>.
    class TwistControl : public RobotControl<geometry_msgs::msg::Twist>
    {
    public:
        explicit TwistControl(ControlConfiguration controlConfiguration)
            : RobotControl<geometry_msgs::msg::Twist>{std::move(controlConfiguration)} {}
    private:
        void BroadcastBus(const geometry_msgs::msg::Twist& message) override;
        void ApplyControl(const geometry_msgs::msg::Twist& message) override;
    };
}  // namespace ROS2
