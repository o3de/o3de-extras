/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/base.h>

namespace ROS2::URDF
{
    // Here is the recommended, minimal number of iterations for position and velocity solver.
    // It is needed since currently O3DE default values are optimized for the gaming experience, not a simulation.
    constexpr AZ::u8 DefaultNumberPosSolver = 40;
    constexpr AZ::u8 DefaultNumberVelSolver = 10;
} // namespace ROS2::URDF
