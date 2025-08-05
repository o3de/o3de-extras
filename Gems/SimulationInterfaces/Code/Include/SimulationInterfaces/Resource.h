/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once
#include <AzCore/std/string/string.h>

namespace SimulationInterfaces
{
    //! A message type to represent simulation Resource
    //! @see <a href="https://github.com/ros-simulation/simulation_interfaces/blob/main/msg/Resource.msg">Resource.msg</a>
    struct Resource
    {
        Resource() = default;
        AZStd::string m_uri;
        AZStd::string m_resourceString;
    };
} // namespace SimulationInterfaces
