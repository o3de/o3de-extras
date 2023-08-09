/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#include <Lidar/LidarSystem.h>
#include <ROS2/Lidar/LidarRegistrarBus.h>

namespace ROS2
{
    LidarSystem::LidarSystem(LidarSystem&& lidarSystem)
        : m_lidars{ AZStd::move(lidarSystem.m_lidars) }
    {
        lidarSystem.BusDisconnect();
    }

    LidarSystem& LidarSystem::operator=(LidarSystem&& lidarSystem)
    {
        lidarSystem.BusDisconnect();
        return lidarSystem;
    }

    void LidarSystem::Activate()
    {
        static constexpr const char* Description = "Collider-based lidar implementation that uses the PhysX engine's raycasting.";
        static constexpr auto SupportedFeatures =
            aznumeric_cast<LidarSystemFeatures>(LidarSystemFeatures::CollisionLayers | LidarSystemFeatures::MaxRangePoints);

        LidarSystemRequestBus::Handler::BusConnect(AZ_CRC(SystemName));

        auto* lidarRegistrarInterface = ROS2::LidarRegistrarInterface::Get();
        AZ_Assert(lidarRegistrarInterface != nullptr, "The Lidar Registrar interface was inaccessible.");

        if (!lidarRegistrarInterface->GetLidarSystemMetaData(SystemName))
        {
            lidarRegistrarInterface->RegisterLidarSystem(SystemName, Description, SupportedFeatures);
        }
    }

    void LidarSystem::Deactivate()
    {
        if (LidarSystemRequestBus::Handler::BusIsConnectedId(AZ_CRC(SystemName)))
        {
            LidarSystemRequestBus::Handler::BusDisconnect();
        }
    }

    LidarId LidarSystem::CreateLidar(AZ::EntityId lidarEntityId)
    {
        LidarId lidarId = LidarId::CreateRandom();
        m_lidars.emplace(lidarId, LidarRaycaster(lidarId, lidarEntityId));
        return lidarId;
    }

    void LidarSystem::DestroyLidar(LidarId lidarId)
    {
        m_lidars.erase(lidarId);
    }
} // namespace ROS2
