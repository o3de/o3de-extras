/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/Math/Color.h>
#include <AzCore/Math/Quaternion.h>
#include <AzCore/Math/Transform.h>
#include <AzCore/Math/Vector3.h>
#include <gz/math/Color.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>

namespace ROS2RobotImporter::Utils::SDFormat
{
    //! Common types conversion between gz::math (libsdformat) and AZ formats
    namespace TypeConversions
    {
        AZ::Vector3 ConvertVector3(const gz::math::Vector3d& gzVector);
        AZ::Quaternion ConvertQuaternion(const gz::math::Quaterniond& gzQuaternion);
        AZ::Color ConvertColor(const gz::math::Color& color);
        AZ::Transform ConvertPose(const gz::math::Pose3d& pose);
    }; // namespace TypeConversions
} // namespace ROS2RobotImporter::Utils::SDFormat
