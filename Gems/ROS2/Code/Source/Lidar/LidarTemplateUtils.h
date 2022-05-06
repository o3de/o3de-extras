/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include "LidarTemplate.h"
#include <AzCore/std/containers/vector.h>

namespace ROS2
{
    class LidarTemplateUtils
    {
    public:
        static LidarTemplate GetTemplate(LidarTemplate::LidarModel model);
        static size_t TotalPointCount(const LidarTemplate& t);
        //! Root rotation as Euler angles in radians.
        static AZStd::vector<AZ::Vector3> PopulateRayDirections(LidarTemplate::LidarModel model,
                                                                const AZ::Vector3& rootRotation);
    };
} // namespace ROS2
