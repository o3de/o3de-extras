/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include "URDFModifications.h"
#include <AzCore/std/containers/vector.h>
#include <AzCore/std/string/string.h>

namespace ROS2::Utils
{
    //! Modifies in memory URDF to increase chance of successful conversion to SDF.
    //! It does the following:
    //! - Adds missing inertia to links of mass 1 kg and identity inertia matrix.
    //! - Renames joints that have the same name as a link.
    //! @param urdf URDF to modify.
    //! @returns a modified URDF and a list of XML element that were modified
    AZStd::pair<std::string, UrdfModifications> ModifyURDFInMemory(const std::string& data);
    AZStd::pair<std::string, UrdfModifications> ModifyURDFInMemory(const AZStd::string& data);

} // namespace ROS2::Utils
