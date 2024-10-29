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
    //! A class for executing lidar raycast.
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
        void ConfigureRayRange(RayRange range) override;
        void ConfigureRaycastResultFlags(RaycastResultFlags flags) override;

        AZ::Outcome<RaycastResults, const char*> PerformRaycast(const AZ::Transform& lidarTransform) override;

        void ConfigureIgnoredCollisionLayers(const AZStd::unordered_set<AZ::u32>& layerIndices) override;
        void ConfigureNonHitReturn(bool returnNonHits) override;

    private:
        static int32_t CompressEntityId(AZ::EntityId entityId);

        AzPhysics::SceneQueryRequests prepareRequests(
            const AZ::Transform& lidarTransform, const AZStd::vector<AZ::Vector3>& rayDirections) const;
        [[nodiscard]] uint8_t GetClassIdForEntity(AZ::EntityId entityId);

        LidarId m_busId;
        //! EntityId that is used to acquire the physics scene handle.
        AZ::EntityId m_sceneEntityId;
        AzPhysics::SceneHandle m_sceneHandle{ AzPhysics::InvalidSceneHandle };

        RaycastResultFlags m_resultFlags{ RaycastResultFlags::Point };
        AZStd::optional<RayRange> m_range{};
        bool m_returnNonHits{ false };
        AZStd::vector<AZ::Quaternion> m_rayRotations{};

        AZStd::unordered_set<AZ::u32> m_ignoredCollisionLayers;
        AZStd::unordered_map<AZ::EntityId, uint8_t> m_entityIdToClassIdCache;
    };
} // namespace ROS2
