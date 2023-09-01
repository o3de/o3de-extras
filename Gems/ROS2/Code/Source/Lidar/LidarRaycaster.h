/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Component/EntityId.h>
#include <AzCore/Math/Transform.h>
#include <AzCore/Math/Vector3.h>
#include <AzCore/std/containers/vector.h>
#include <AzFramework/Physics/PhysicsScene.h>
#include <ROS2/Lidar/LidarRaycasterBus.h>

namespace ROS2
{
    class LidarRaycaster : protected LidarRaycasterRequestBus::Handler
    {
    public:
        LidarRaycaster(LidarId busId, AZ::EntityId sceneEntityId);
        LidarRaycaster(LidarRaycaster&& lidarSystem);
        LidarRaycaster(const LidarRaycaster& lidarSystem) = default;
        ~LidarRaycaster() override;

    protected:
        // LidarRaycasterRequestBus overrides
        void ConfigureRayOrientations(const AZStd::vector<AZ::Vector3>& orientations) override;
        void ConfigureRayRange(float range) override;
        void ConfigureMinimumRayRange(float range) override;
        void ConfigureRaycastResultFlags(RaycastResultFlags flags) override;

        RaycastResult PerformRaycast(const AZ::Transform& lidarTransform) override;

        void ConfigureIgnoredCollisionLayers(const AZStd::unordered_set<AZ::u32>& layerIndices) override;
        void ConfigureMaxRangePointAddition(bool addMaxRangePoints) override;

    private:
        AzPhysics::SceneQueryRequests prepareRequests(
            const AZ::Transform& lidarTransform, const AZStd::vector<AZ::Vector3>& rayDirections) const;
        LidarId m_busId;
        //! EntityId that is used to acquire the physics scene handle.
        AZ::EntityId m_sceneEntityId;
        AzPhysics::SceneHandle m_sceneHandle{ AzPhysics::InvalidSceneHandle };

        RaycastResultFlags m_resultFlags{ RaycastResultFlags::Points };
        float m_minRange{ 0.0f };
        float m_range{ 1.0f };
        bool m_addMaxRangePoints{ false };
        AZStd::vector<AZ::Vector3> m_rayRotations{ { AZ::Vector3::CreateZero() } };

        AZStd::unordered_set<AZ::u32> m_ignoredCollisionLayers;
    };
} // namespace ROS2
