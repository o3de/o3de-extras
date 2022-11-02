/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "RobotImporterUtils.h"
#include "TypeConversions.h"
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

    AZ::Transform Utils::getWorldTransformURDF(const urdf::LinkSharedPtr& link, AZ::Transform t)
    {
        if (link->getParent() != nullptr)
        {
            t = URDF::TypeConversions::ConvertPose(link->parent_joint->parent_to_joint_origin_transform) * t;
            return getWorldTransformURDF(link->getParent(), t);
        }
        return t;
    }

    AZStd::unordered_map<AZStd::string, urdf::LinkSharedPtr> Utils::getAllLinks(const std::vector<urdf::LinkSharedPtr>& child_links)
    {
        AZStd::unordered_map<AZStd::string, urdf::LinkSharedPtr> pointers;
        std::function<void(const std::vector<urdf::LinkSharedPtr>&)> link_visitor =
            [&](const std::vector<urdf::LinkSharedPtr>& child_links) -> void
        {
            for (auto child_link : child_links)
            {
                AZStd::string link_name(child_link->name.c_str(), child_link->name.size());
                pointers[link_name] = child_link;
                link_visitor(child_link->child_links);
            }
        };
        link_visitor(child_links);
        return pointers;
    }

    AZStd::unordered_map<AZStd::string, urdf::JointSharedPtr> Utils::getAllJoints(const std::vector<urdf::LinkSharedPtr>& child_links)
    {
        AZStd::unordered_map<AZStd::string, urdf::JointSharedPtr> joints;
        std::function<void(const std::vector<urdf::LinkSharedPtr>&)> link_visitor =
            [&](const std::vector<urdf::LinkSharedPtr>& child_links) -> void
        {
            for (auto child_link : child_links)
            {
                const auto& joint = child_link->parent_joint;
                AZStd::string joint_name(joint->name.c_str(), joint->name.size());
                joints[joint_name] = joint;
                link_visitor(child_link->child_links);
            }
        };
        link_visitor(child_links);
        return joints;
    }

} // namespace ROS2
