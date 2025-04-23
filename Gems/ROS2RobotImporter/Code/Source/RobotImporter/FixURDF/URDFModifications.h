/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once
#include <AzCore/std/containers/vector.h>
#include <AzCore/std/string/string.h>

namespace ROS2::Utils
{
    struct MissingInertia
    {
        AZStd::string linkName;

        ~MissingInertia() = default;
    };

    struct IncompleteInertia
    {
        AZStd::string linkName;
        AZStd::vector<AZStd::string> missingTags;
    };

    struct DuplicatedJoint
    {
        AZStd::string oldName;
        AZStd::string newName;
    };

    struct UrdfModifications
    {
        AZStd::vector<MissingInertia> missingInertias;
        AZStd::vector<IncompleteInertia> incompleteInertias;
        AZStd::vector<DuplicatedJoint> duplicatedJoints;
    };

} // namespace ROS2::Utils
