/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/RTTI/TypeInfo.h>

namespace ROS2
{
    //! Abstracted from ROS message: http://docs.ros.org/en/api/ackermann_msgs/html/msg/AckermannDrive.html
    struct AckermannCommandStruct
    {
        AZ_TYPE_INFO(AckermannCommandStruct, "{6D03C30F-F06B-4CEE-8AD1-DDCCCB57C4B5}");
        float m_steeringAngle = 0; //!< desired virtual angle (radians)
        float m_steeringAngleVelocity = 0; //!< desired rate of change (radians/s)
        float m_speed = 0; //!< desired forward speed (m/s)
        float m_acceleration = 0; //!< desired acceleration (m/s^2)
        float m_jerk = 0; //!< desired jerk (m/s^3)
    };
} // namespace ROS2