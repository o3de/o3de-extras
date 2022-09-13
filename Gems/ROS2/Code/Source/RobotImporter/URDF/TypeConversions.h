/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include "UrdfParser.h"
#include <AzCore/Math/Color.h>
#include <AzCore/Math/Quaternion.h>
#include <AzCore/Math/Vector3.h>

namespace ROS2::URDF
{
    //! Common types conversion between urdf and AZ formats
    class TypeConversions
    {
    public:
        static AZ::Vector3 ConvertVector3(const urdf::Vector3& urdfVector);
        static AZ::Quaternion ConvertQuaternion(const urdf::Rotation& urdfQuaternion);
        static AZ::Color ConvertColor(const urdf::Color& color);
    };
} // namespace ROS2::URDF
