/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "TypeConversions.h"

namespace ROS2RobotImporter::Utils
{
    AZ::Vector3 TypeConversions::ConvertVector3(const gz::math::Vector3d& gzVector)
    {
        return AZ::Vector3(gzVector.X(), gzVector.Y(), gzVector.Z());
    }

    AZ::Quaternion TypeConversions::ConvertQuaternion(const gz::math::Quaterniond& gzQuaternion)
    {
        return AZ::Quaternion(gzQuaternion.X(), gzQuaternion.Y(), gzQuaternion.Z(), gzQuaternion.W());
    }

    AZ::Color TypeConversions::ConvertColor(const gz::math::Color& color)
    {
        return AZ::Color(color.R(), color.G(), color.B(), color.A());
    }

    AZ::Transform TypeConversions::ConvertPose(const gz::math::Pose3d& pose)
    {
        AZ::Quaternion azRotation = Utils::TypeConversions::ConvertQuaternion(pose.Rot());
        AZ::Vector3 azPosition = Utils::TypeConversions::ConvertVector3(pose.Pos());
        return AZ::Transform(azPosition, azRotation, 1.0f);
    }

} // namespace ROS2RobotImporter::Utils
