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
    AZ::Vector3 TypeConversions::ConvertVector3(const urdf::Vector3& urdfVector)
    {
        return AZ::Vector3(urdfVector.x, urdfVector.y, urdfVector.z);
    }

    AZ::Quaternion TypeConversions::ConvertQuaternion(const urdf::Rotation& urdfQuaternion)
    {
        return AZ::Quaternion(urdfQuaternion.x, urdfQuaternion.y, urdfQuaternion.z, urdfQuaternion.w);
    }

    AZ::Color TypeConversions::ConvertColor(const urdf::Color& color)
    {
        return AZ::Color(color.r, color.g, color.b, color.a);
    }
} // namespace ROS2::URDF
