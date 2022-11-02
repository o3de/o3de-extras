/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include "AzCore/Component/ComponentBus.h"
#include "AzCore/std/string/string.h"
#include "RobotImporter/URDF/UrdfParser.h"
#include <AzCore/Math/Transform.h>
#include <AzCore/std/containers/unordered_map.h>
namespace ROS2
{
    namespace Utils
    {
        bool IsWheelURDFHeuristics(const urdf::LinkConstSharedPtr& link);

        /// Goes through URDF and finds world to entity transformation for us
        AZ::Transform getWorldTransformURDF(const urdf::LinkSharedPtr& link, AZ::Transform t = AZ::Transform::Identity());

        /// Retrieve all links in urdf file
        AZStd::unordered_map<AZStd::string, urdf::LinkSharedPtr> getAllLinks(const std::vector<urdf::LinkSharedPtr>& links);

        /// Retrieve all links in urdf file
        AZStd::unordered_map<AZStd::string, urdf::JointSharedPtr> getAllJoints(const std::vector<urdf::LinkSharedPtr>& links);

    } // namespace Utils
} // namespace ROS2