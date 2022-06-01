/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Component/EntityId.h>
#include <AzCore/Math/Vector3.h>
#include <AzCore/std/containers/vector.h>

// TODO - switch to interface
namespace ROS2
{
    //! A simple implementation of Lidar operation in terms of raycasting.
    class LidarRaycaster
    {
    public:
        //! Perform raycast against the current scene.
        //! @param start Starting point of rays. This is a simplification since there can be multiple starting points
        //! in real sensors.
        //! @param directions Directions in which to shoot rays. These should be generated from Lidar configuration.
        //! @param distance Maximum distance for ray-casting.
        //! @return Hits of raycast. The returned vector size can be anything between zero and size of directions.
        //! No hits further than distance will be reported.
        // TODO - different starting points for rays, distance from reference point, noise models, rotating mirror sim, other
        // TODO - customized settings. Encapsulate in lidar definition and pass in constructor, update transform.
        AZStd::vector<AZ::Vector3> PerformRaycast(const AZ::Vector3& start, const AZStd::vector<AZ::Vector3>& directions, float distance);

        void setSelfColliderEntity(const AZ::EntityId &selfColliderEntity);

    private:
        AZ::EntityId m_selfColliderEntityId;
    };
}  // namespace ROS2
