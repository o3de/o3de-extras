/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <Lidar/LidarRaycaster.h>
#include <ROS2/Lidar/LidarSystemBus.h>

namespace ROS2
{
    class LidarSystem : protected ROS2::LidarSystemRequestBus::Handler
    {
    public:
        LidarSystem() = default;
        LidarSystem(LidarSystem&& lidarSystem);
        LidarSystem& operator=(LidarSystem&& lidarSystem);
        LidarSystem(const LidarSystem& lidarSystem) = default;
        LidarSystem& operator=(const LidarSystem& lidarSystem) = default;

        ~LidarSystem() = default;

        void Activate();
        void Deactivate();

        static constexpr const char* SystemName = "Scene Queries";

    private:
        // LidarSystemRequestBus overrides
        LidarId CreateLidar(AZ::EntityId lidarEntityId) override;
        void DestroyLidar(LidarId lidarId) override;

        AZStd::unordered_map<LidarId, LidarRaycaster> m_lidars;
    };
} // namespace ROS2
