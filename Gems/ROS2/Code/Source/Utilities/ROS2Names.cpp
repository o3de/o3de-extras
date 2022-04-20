/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ROS2Names.h"
#include <AzCore/std/string/regex.h>

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

    AZStd::string ROS2Names::RosifyName(const AZStd::string& input)
    {
        // TODO - add unit tests

        // TODO - implement stricter guidelines and differentiate: https://design.ros2.org/articles/topic_and_service_names.html
        // TODO - add check whether it begins with a number (and if so, prepend underscore)

        AZStd::string rosified;
        const AZStd::regex ros2Disallowedlist("[^0-9|a-z|A-Z|_]");
        return AZStd::regex_replace(input, ros2Disallowedlist, "_");
    }
}  // namespace ROS2