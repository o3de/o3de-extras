/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once
#include "Resource.h"
#include <AzCore/std/containers/vector.h>
#include <AzCore/std/string/string.h>

namespace SimulationInterfaces
{
    //! A message type to represent simulation Resource
    //! @see <a href="https://github.com/ros-simulation/simulation_interfaces/blob/main/msg/WorldResource.msg">Resource.msg</a>
    struct WorldResource
    {
        WorldResource() = default;
        WorldResource(AZStd::string name, Resource resource, AZStd::string description, AZStd::vector<AZStd::string> tags)
            : m_name(name)
            , m_worldResource(resource)
            , m_description(description)
            , m_tags(tags)
        {
        }
        AZStd::string m_name;
        Resource m_worldResource;
        AZStd::string m_description;
        AZStd::vector<AZStd::string> m_tags;
    };
} // namespace SimulationInterfaces
