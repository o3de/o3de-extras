/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include "ITimeSource.h"

namespace ROS2
{
    //! The ROS2TimeSource provides the time taken from the ROS 2. By default
    //! this is system_time but it can be configured to provide the time from the other sources.
    class ROS2TimeSource : public ITimeSource
    {
    public:
        virtual ~ROS2TimeSource() = default;

        // ITimeSource overrides ...
        virtual void Activate() override {};
        virtual void Deactivate() override {};

        //! Get ROS 2 time as ROS2 message.
        //! @see ROS2Requests::GetROSTimestamp() for more details.
        virtual builtin_interfaces::msg::Time GetROSTimestamp() const override;
    };

} // namespace ROS2
