/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Debug/Trace.h>
#include <AzCore/Settings/SettingsRegistry.h>
#include <AzCore/std/string/string.h>
#include <AzCore/std/string/string_view.h>

namespace ROS2
{
    //! How event source adapters (ROS2::EventSourceAdapter) pace their adapted events, chosen with the registry key
    //! SensorPacingRegistryKey:
    //!  - "frames" (default): count source events. After each adapted event, wait round(estimated source rate / frequency) source
    //!    events; tick-based sources estimate their rate from the median frame time. Frame drops do not shorten the wait, so a sensor
    //!    that is costly compared with its period slows down instead of the frame rate. The adapted frequency is approximate: the
    //!    rounding and a quantized frame time (for example from a /clock published in steps) can move it by tens of percent.
    //!  - "simulation_time": accumulate the source's delta time (for tick-based sources, the ROS time since the previous tick) and
    //!    signal on the source event closest to each deadline, keeping the remainder. The adapted frequency is then the set one on
    //!    average, also when the source rate is not a multiple of it; a costly sensor costs its full price every period. After a
    //!    stall longer than two periods the backlog is dropped, so at most one adapted event follows a frame drop early.
    enum class SensorPacing
    {
        Frames,
        SimulationTime,
    };

    inline constexpr AZStd::string_view SensorPacingRegistryKey = "/O3DE/ROS2/SensorPacing";

    //! The pacing set in the registry, or SensorPacing::Frames when it is not set.
    [[nodiscard]] inline SensorPacing GetSensorPacingSetting()
    {
        AZStd::string value;
        auto* registry = AZ::SettingsRegistry::Get();
        if (registry == nullptr || !registry->Get(value, SensorPacingRegistryKey) || value == "frames")
        {
            return SensorPacing::Frames;
        }
        if (value == "simulation_time")
        {
            return SensorPacing::SimulationTime;
        }
        AZ_Warning(
            "SensorPacing",
            false,
            "Unknown %.*s \"%s\"; expected \"frames\" or \"simulation_time\". Using \"frames\".",
            AZ_STRING_ARG(SensorPacingRegistryKey),
            value.c_str());
        return SensorPacing::Frames;
    }
} // namespace ROS2
