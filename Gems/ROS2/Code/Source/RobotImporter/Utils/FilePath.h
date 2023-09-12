/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzToolsFramework/API/ToolsApplicationAPI.h>

namespace ROS2::Utils
{
    //! Returns true if the specified file path extension is .xacro
    bool IsFileXacro(AZ::IO::PathView filename);
    //! Returns true if the specified file path extension is .urdf
    bool IsFileUrdf(AZ::IO::PathView filename);
    //! Returns true if the specified file path extension is .sdf or .world
    bool IsFileSdf(AZ::IO::PathView filename);

    //! Returns true if file is either a URDF or SDF file
    bool IsFileUrdfOrSdf(AZ::IO::PathView filename);

    //! Returns true if file is either a Xacro, URDF or SDF file
    bool IsFileXacroOrUrdfOrSdf(AZ::IO::PathView filename);
} // namespace ROS2::Utils
