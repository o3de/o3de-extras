/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/std/containers/unordered_set.h>
#include <AzCore/std/string/string.h>

namespace SimulationInterfaces
{
    //! Structure to design a filter for tags
    //!  @see <a href="https://github.com/ros-simulation/simulation_interfaces/blob/main/msg/TagsFilter.msg">TagsFilter.msg</a>

    struct TagFilter
    {
        enum class TagFilterMode
        {
            FILTER_MODE_ANY,
            FILTER_MODE_ALL
        };
        AZStd::unordered_set<AZStd::string> m_tags;
        TagFilterMode m_mode;
    };

} // namespace SimulationInterfaces
