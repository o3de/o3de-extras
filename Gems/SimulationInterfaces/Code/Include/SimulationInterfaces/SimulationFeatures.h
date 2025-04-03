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

    //! Simulation Features copied from the simulation_interfaces
    //! to avoid ros2 dependency
    //! @see https://github.com/ros-simulation/simulation_interfaces/blob/main/msg/SimulatorFeatures.msg
    enum class SimulationFeatures : AZ::u8
    {
        SPAWNING = 0,
        DELETING = 1,
        NAMED_POSES = 2,
        POSE_BOUNDS = 3,
        ENTITY_TAGS = 4,
        ENTITY_BOUNDS = 5,
        ENTITY_BOUNDS_BOX = 6,
        ENTITY_BOUNDS_CONVEX = 7,
        ENTITY_CATEGORIES = 8,
        SPAWNING_RESOURCE_STRING = 9,
        ENTITY_STATE_GETTING = 10,
        ENTITY_STATE_SETTING = 11,
        ENTITY_INFO_GETTING = 12,
        ENTITY_INFO_SETTING = 13,
        SPAWNABLES = 14,
        SIMULATION_RESET = 20,
        SIMULATION_RESET_TIME = 21,
        SIMULATION_RESET_STATE = 22,
        SIMULATION_RESET_SPAWNED = 23,
        SIMULATION_STATE_GETTING = 24,
        SIMULATION_STATE_SETTING = 25,
        SIMULATION_STATE_PAUSE = 26,
        STEP_SIMULATION_SINGLE = 31,
        STEP_SIMULATION_MULTIPLE = 32,
        STEP_SIMULATION_ACTION = 33
    };
} // namespace SimulationInterfaces