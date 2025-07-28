/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <AzCore/Time/ITime.h>
#include "RealTimeSource.h"

namespace ROS2
{
    builtin_interfaces::msg::Time RealTimeSource::GetROSTimestamp() const
    {
        const auto elapsedTime = GetElapsedTimeMicroseconds();

        builtin_interfaces::msg::Time timeStamp;
        timeStamp.sec = static_cast<int32_t>(elapsedTime / 1000000);
        timeStamp.nanosec = static_cast<uint32_t>((elapsedTime % 1000000) * 1000);
        return timeStamp;
    }

    int64_t RealTimeSource::GetElapsedTimeMicroseconds() const
    {
        if (auto* timeSystem = AZ::Interface<AZ::ITime>::Get())
        {
            return static_cast<int64_t>(timeSystem->GetElapsedTimeUs());
        }
        else
        {
            AZ_Error("RealTimeSource", false, "No ITime interface available for ROS2 Gem simulation clock");
            return 0;
        }
    }

    AZ::Outcome<void, AZStd::string> RealTimeSource::AdjustTime(const builtin_interfaces::msg::Time& time)
    {
        return AZ::Failure(AZStd::string("RealTimeSource does not support setting a specific time."));
    }


} // namespace ROS2
