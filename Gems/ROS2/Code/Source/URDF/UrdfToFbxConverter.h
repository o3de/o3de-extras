/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/std/string/string.h>

namespace ROS2
{
    //! Class for conversion from URDF to Filmbox (.fbx) files
    class UrdfToFbxConverter
    {
    public:
        AZStd::string ConvertUrdfToFbx(const AZStd::string & urdfString);
    };

} // namespace ROS2