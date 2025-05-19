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
    //! The RealTimeSource starts from 0 at the start of the simulation.
    //! This time source could be affected by the jitter in the data, simulation
    //! computations or other similar events. On the other hand RealTimeSource
    //! can remain consistent with the other independent clocks if it is synchronized
    //! (e.g. through NTP).
    class RealTimeSource : public ITimeSource
    {
    public:
        virtual ~RealTimeSource() = default;

        // ITimeSource overrides ...
        virtual void Activate() override{};
        virtual void Deactivate() override{};

        //! Get simulation time as ROS 2 message.
        //! @see ROS2Requests::GetROSTimestamp() for more details.
        virtual builtin_interfaces::msg::Time GetROSTimestamp() const override;

    private:
        //! Get the time since start of sim, scaled with t_simulationTickScale
        int64_t GetElapsedTimeMicroseconds() const;
    };
} // namespace ROS2
