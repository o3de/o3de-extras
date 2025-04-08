/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/base.h>

namespace SimulationInterfaces
{

    //! Simulation States copied from the simulation_interfaces
    //! to avoid ros2 dependency
    //! @see https://github.com/ros-simulation/simulation_interfaces/blob/main/msg/SimulationState.msg
    enum class SimulationStates : AZ::u8
    {
        STATE_STOPPED = 0,
        STATE_PLAYING = 1,
        STATE_PAUSED = 2,
        STATE_QUITTING = 3
    };

} // namespace SimulationInterfaces
