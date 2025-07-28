/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <builtin_interfaces/msg/time.hpp>
#include <AzCore/Outcome/Outcome.h>
#include <AzCore/std/string/string.h>

namespace ROS2
{
    //! Interface for time source used by ROS2ClockSystemComponent.
    class ITimeSource
    {
    public:
        virtual void Activate() = 0;
        virtual void Deactivate() = 0;

        virtual ~ITimeSource() = default;

        //! Sets the time source to the given time.
        //! @param time The time to set the time source to.
        //! @return An outcome indicating success or failure.
        virtual AZ::Outcome<void, AZStd::string> AdjustTime(const builtin_interfaces::msg::Time & time) = 0;

        //! Get time as ROS 2 message.
        //! @see ROS2Requests::GetROSTimestamp() for more details.
        virtual builtin_interfaces::msg::Time GetROSTimestamp() const = 0;
    };
} // namespace ROS2
