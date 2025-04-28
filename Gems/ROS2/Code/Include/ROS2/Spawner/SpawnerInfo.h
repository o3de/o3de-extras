/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Math/Transform.h>
#include <AzCore/Memory/Memory.h>
#include <AzCore/Memory/Memory_fwd.h>
#include <AzCore/Memory/SystemAllocator.h>
#include <AzCore/std/containers/unordered_map.h>
#include <AzCore/std/string/string.h>

namespace ROS2
{
    struct SpawnPointInfo
    {
        AZStd::string info;
        AZ::Transform pose;
    };

    using SpawnPointInfoMap = AZStd::unordered_map<AZStd::string, SpawnPointInfo>;
} // namespace ROS2
