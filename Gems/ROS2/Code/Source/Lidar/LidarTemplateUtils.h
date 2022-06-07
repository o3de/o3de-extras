/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include "LidarTemplate.h"
#include <AzCore/Math/Vector3.h>
#include <AzCore/std/containers/vector.h>

namespace ROS2
{
    //! Utility class for Lidar model computations.
    class LidarTemplateUtils
    {
    public:
        static LidarTemplate GetTemplate(LidarTemplate::LidarModel model);
        static size_t TotalPointCount(const LidarTemplate& t);

        //! Compute ray directions based on lidar model and rotation.
        //! @param model Lidar model to use. Note that different models will produce different number of rays.
        //! @param rootRotation Root rotation as Euler angles in radians.
        //! @return All ray directions which can be used to perform ray-casting simulation of lidar operation.
        static AZStd::vector<AZ::Vector3> PopulateRayDirections(const LidarTemplate& lidarTemplate,
                                                                const AZ::Vector3& rootRotation);
    };
} // namespace ROS2
