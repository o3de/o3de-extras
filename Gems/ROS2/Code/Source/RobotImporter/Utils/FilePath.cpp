/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "FilePath.h"

namespace ROS2
{
    namespace Utils
    {
        //! @returns the capitalized extention of provided file.
        //! In the case that the file does not have an extension an empty string will be returned.
        AZStd::string GetCapitalizedExtension(const AZ::IO::Path& filename)
        {
            if (!filename.HasExtension())
            {
                return "";
            }
            AZStd::string extension{ filename.Extension().Native() };
            AZStd::to_upper(extension.begin(), extension.end());
            return extension;
        }

        bool IsFileXacro(const AZ::IO::Path& filename)
        {
            return GetCapitalizedExtension(filename) == ".XACRO";
        }

        bool IsFileUrdf(const AZ::IO::Path& filename)
        {
            return GetCapitalizedExtension(filename) == ".URDF";
        }

        bool IsFileSDF(const AZ::IO::Path& filename)
        {
            return GetCapitalizedExtension(filename) == ".SDF";
        }
    } // namespace Utils
} // namespace ROS2
