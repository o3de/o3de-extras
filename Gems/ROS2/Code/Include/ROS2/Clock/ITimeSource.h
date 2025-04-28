/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <builtin_interfaces/msg/time.hpp>

namespace ROS2
{
    class ITimeSource
    {
    public:
        virtual void Activate() = 0;
        virtual void Deactivate() = 0;

        virtual ~ITimeSource() = default;

        //! Get time as ROS 2 message.
        //! @see ROS2Requests::GetROSTimestamp() for more details.
        virtual builtin_interfaces::msg::Time GetROSTimestamp() const = 0;
    };
} // namespace ROS2
