/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/Math/Vector3.h>
#include <AzCore/Outcome/Outcome.h>
#include <AzCore/std/containers/vector.h>
#include <AzCore/std/string/string.h>
#include <simulation_interfaces/msg/bounds.hpp>

namespace SimulationInterfaces
{
    //! Bounds types to be used in the Bounds message
    //!  @see <a href="https://github.com/ros-simulation/simulation_interfaces/blob/main/msg/Bounds.msg">Result.msg</a>
    using BoundsType = simulation_interfaces::msg::Bounds::_type_type;

    //! A message type to represent the bounds
    struct Bounds
    {
        Bounds() = default;
        Bounds(BoundsType boundsType, const AZStd::vector<AZ::Vector3>& points)
            : m_boundsType(boundsType)
            , m_points(points)
        {
        }
        BoundsType m_boundsType;
        AZStd::vector<AZ::Vector3> m_points;
    };
} // namespace SimulationInterfaces
