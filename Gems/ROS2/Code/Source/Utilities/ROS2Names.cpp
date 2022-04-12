/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ROS2Names.h"

namespace ROS2
{
    AZStd::string ROS2Names::GetNamespacedName(const AZStd::string& ns, const AZStd::string& name)
    {
        if (ns.empty())
        {
            return name;

        }
        return AZStd::string::format("%s/%s", ns.c_str(), name.c_str());;
    }
}  // namespace ROS2