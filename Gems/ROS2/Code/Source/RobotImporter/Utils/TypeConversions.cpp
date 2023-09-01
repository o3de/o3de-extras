/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "TypeConversions.h"

namespace ROS2::URDF
{
    AZ::Vector3 TypeConversions::ConvertVector3(const gz::math::Vector3d& urdfVector)
    {
        return AZ::Vector3(urdfVector.X(), urdfVector.Y(), urdfVector.Z());
    }

    AZ::Quaternion TypeConversions::ConvertQuaternion(const gz::math::Quaterniond& urdfQuaternion)
    {
        return AZ::Quaternion(urdfQuaternion.X(), urdfQuaternion.Y(), urdfQuaternion.Z(), urdfQuaternion.W());
    }

    AZ::Color TypeConversions::ConvertColor(const gz::math::Color& color)
    {
        return AZ::Color(color.R(), color.G(), color.B(), color.A());
    }

    AZ::Transform TypeConversions::ConvertPose(const gz::math::Pose3d& pose)
    {
        AZ::Quaternion azRotation = URDF::TypeConversions::ConvertQuaternion(pose.Rot());
        AZ::Vector3 azPosition = URDF::TypeConversions::ConvertVector3(pose.Pos());
        return AZ::Transform(azPosition, azRotation, 1.0f);
    }

} // namespace ROS2::URDF
