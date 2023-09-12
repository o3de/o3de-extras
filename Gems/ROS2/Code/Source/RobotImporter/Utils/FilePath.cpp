/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "FilePath.h"

namespace ROS2::Utils
{
    //! @returns the capitalized extention of provided file.
    //! In the case that the file does not have an extension an empty string will be returned.
    AZ::IO::FixedMaxPathString GetCapitalizedExtension(AZ::IO::PathView filename)
    {
        if (!filename.HasExtension())
        {
            return AZ::IO::FixedMaxPathString{};
        }
        AZ::IO::FixedMaxPathString extension{ filename.Extension().Native() };
        AZStd::to_upper(extension.begin(), extension.end());
        return extension;
    }

    bool IsFileXacro(AZ::IO::PathView filename)
    {
        return GetCapitalizedExtension(filename) == ".XACRO";
    }

    bool IsFileUrdf(AZ::IO::PathView filename)
    {
        return GetCapitalizedExtension(filename) == ".URDF";
    }

    bool IsFileSdf(AZ::IO::PathView filename)
    {
        return GetCapitalizedExtension(filename) == ".SDF" || GetCapitalizedExtension(filename) == ".WORLD";
    }

    bool IsFileUrdfOrSdf(AZ::IO::PathView filename)
    {
        return IsFileUrdf(filename) || IsFileSdf(filename);
    }

    bool IsFileXacroOrUrdfOrSdf(AZ::IO::PathView filename)
    {
        return IsFileXacro(filename) || IsFileUrdf(filename) || IsFileSdf(filename);
    }
} // namespace ROS2::Utils
