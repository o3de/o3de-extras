/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzToolsFramework/API/ToolsApplicationAPI.h>

namespace ROS2
{
    namespace Utils
    {
        AZStd::string GetCapitalizedExtension(const AZ::IO::Path& filename);

        bool IsFileXacro(const AZ::IO::Path& filename);

        bool IsFileUrdf(const AZ::IO::Path& filename);
    } // namespace Utils
} // namespace ROS2
