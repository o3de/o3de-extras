/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Math/Vector3.h>
#include <AzCore/std/containers/vector.h>
#include <Lidar/LidarTemplate.h>

namespace ROS2
{
    //! Utility class for Lidar model computations.
    namespace LidarTemplateUtils
    {
        //! Get the lidar template for a model.
        //! @param model lidar model.
        //! @return the matching template which describes parameters for the model.
        LidarTemplate GetTemplate(LidarTemplate::LidarModel model);

        //! Get all 2D lidar models.
        //! @return 2D lidar models.
        AZStd::vector<LidarTemplate::LidarModel> Get2DModels();

        //! Get all 3D lidar models.
        //! @return 3D lidar models.
        AZStd::vector<LidarTemplate::LidarModel> Get3DModels();

        //! Get total point count for a given template.
        //! @param t lidar template.
        //! @return total count of points that the lidar specified by the template would produce on each scan.
        size_t TotalPointCount(const LidarTemplate& t);

        //! Compute ray Rotation angles based on lidar model.
        //! @param lidarTemplate Lidar model to use. Note that different models will produce different number of rays.
        //! @return Ray rotations angles as Euler angles in radians.
        AZStd::vector<AZ::Vector3> PopulateRayRotations(const LidarTemplate& lidarTemplate);

        //! Compute ray directions from rotations.
        //! @param rotations Rotations as Euler angles in radians to compute directions from.
        //! @param rootRotation Root rotation as Euler angles in radians.
        //! @return Ray directions constructed by transforming an X axis unit vector by the provided rotations.
        AZStd::vector<AZ::Vector3> RotationsToDirections(const AZStd::vector<AZ::Vector3>& rotations, const AZ::Transform& rootTransform);
    }; // namespace LidarTemplateUtils
} // namespace ROS2
