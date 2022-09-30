/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "RobotImporterUtils.h"
#include <AzCore/std/string/regex.h>
namespace ROS2
{

    bool Utils::IsWheelURDFHeuristics(const urdf::LinkConstSharedPtr& link)
    {
        const AZStd::regex wheel_regex("wheel[_]||[_]wheel");
        const AZStd::regex joint_regex("(?i)joint");
        const AZStd::string link_name(link->name.c_str(), link->name.size());
        AZStd::smatch match;
        // check is name is catchy for wheel
        if (!AZStd::regex_search(link_name, match, wheel_regex))
        {
            return false;
        }
        // but it should cointain joint word
        if (AZStd::regex_search(link_name, match, joint_regex))
        {
            return false;
        }
        // wheel need to have collision and visuals
        if (!(link->collision && link->visual))
        {
            return false;
        }
        // and finally parent joint needs to be CONTINOUS
        if (link->parent_joint && link->parent_joint->type == urdf::Joint::CONTINUOUS)
        {
            return true;
        }
        return false;
    }

} // namespace ROS2
