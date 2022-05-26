/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "UrdfToFbxConverter.h"

#include <AzCore/std/string/string.h>

#include "UrdfParser.h"
#include "FbxGenerator.h"

namespace ROS2
{
    AZStd::string UrdfToFbxConverter::ConvertUrdfToFbx(const AZStd::string & urdfString)
    {
        // TODO: Add implementation

        // Workflow
        // 1. Parse URDF with UrdfParser
        // 2. Create FBX file with FbxGenerator
        // 3. Add each object from URDF based structure to FBX generator.

        return "";
    }

} // namespace ROS2