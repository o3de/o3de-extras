/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#include "LidarSystem.h"
#include "ROS2/Lidar/LidarRegistrarBus.h"

namespace ROS2
{
    AzPhysics::SceneHandle GetPhysicsSceneFromEntityId(const AZ::EntityId& entityId)
    {
        auto* physicsSystem = AZ::Interface<AzPhysics::SystemInterface>::Get();
        auto foundBody = physicsSystem->FindAttachedBodyHandleFromEntityId(entityId);
        AzPhysics::SceneHandle lidarPhysicsSceneHandle = foundBody.first;
        if (foundBody.first == AzPhysics::InvalidSceneHandle)
        {
            auto* sceneInterface = AZ::Interface<AzPhysics::SceneInterface>::Get();
            lidarPhysicsSceneHandle = sceneInterface->GetSceneHandle(AzPhysics::DefaultPhysicsSceneName);
        }

        AZ_Assert(lidarPhysicsSceneHandle != AzPhysics::InvalidSceneHandle, "Invalid physics scene handle for entity");
        return lidarPhysicsSceneHandle;
    }

    void LidarSystem::Activate()
    {
        const char* name = "PhysX";
        const char* description = "Collider-based lidar implementation that uses the PhysX engine's raycasting.";
        const LidarSystemFeatures supportedFeatures = {
            /* .m_noise =                   */ false,
            /* .m_collisionLayers =         */ true,
            /* .m_entityExclusion =         */ false,
            /* .m_MaxRangeHitPointConfig =  */ true,
        };

        LidarSystemRequestBus::Handler::BusConnect(AZ_CRC(name));

        auto* lidarSystemManagerInterface = ROS2::LidarRegistrarInterface::Get();
        AZ_Assert(lidarSystemManagerInterface != nullptr, "The LidarSystem Manager interface was inaccessible.");
        lidarSystemManagerInterface->RegisterLidarSystem(name, description, supportedFeatures);
    }

    void LidarSystem::Deactivate()
    {
        LidarSystemRequestBus::Handler::BusDisconnect();
    }

    AZ::Uuid LidarSystem::CreateLidar(const AZ::EntityId& lidarEntityId)
    {
        AZ::Uuid lidarUuid = AZ::Uuid::CreateRandom();
        m_lidars.emplace_back(lidarUuid);
        m_lidars.back().SetRaycasterScene(GetPhysicsSceneFromEntityId(lidarEntityId));
        return lidarUuid;
    }
} // namespace ROS2