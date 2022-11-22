/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include "LidarRaycaster.h"
#include "ROS2/Lidar/LidarSystemBus.h"

namespace ROS2
{
    class LidarSystem : protected ROS2::LidarSystemRequestBus::Handler
    {
    public:
        LidarSystem() = default;
        LidarSystem(LidarSystem&& lidarSystem) = delete;
        LidarSystem(const LidarSystem& lidarSystem) = delete;
        ~LidarSystem() = default;

        void Activate();
        void Deactivate();

    private:
        ////////////////////////////////////////////////////////////////////////
        // LidarSystemRequestBus::Handler interface implementation
        AZ::Uuid CreateLidar(const AZ::EntityId& lidarEntityId) override;
        ////////////////////////////////////////////////////////////////////////

        AZStd::vector<LidarRaycaster> m_lidars;
    };
} // namespace ROS2
