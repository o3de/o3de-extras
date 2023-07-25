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

#include <sdf/sdf.hh>

namespace ROS2::SDFormat
{
    namespace SupportedTags
    {
        AZStd::unordered_set<AZStd::string> GetSupportedTags(const sdf::SensorType& sensorType);
    } // namespace SupportedTags
} // namespace ROS2::SDFormat
