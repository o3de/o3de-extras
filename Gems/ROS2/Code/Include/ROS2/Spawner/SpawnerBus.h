/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Component/ComponentBus.h>
#include <AzCore/Component/EntityId.h>
#include <AzCore/EBus/EBus.h>
#include <AzCore/Math/Transform.h>
#include <AzCore/RTTI/BehaviorContext.h>
#include <ROS2/ROS2TypeIds.h>
#include <ROS2/Spawner/SpawnerInfo.h>

namespace ROS2
{
    //!  Interface class allowing requesting Spawner interface from other components.
    class SpawnerRequests : public AZ::ComponentBus
    {
    public:
        AZ_RTTI(SpawnerRequests, SpawnerRequestsTypeId);

        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;

        virtual ~SpawnerRequests() = default;

        //! Default spawn pose getter
        //! @return default spawn point coordinates set by user in Editor (by default: translation: {0, 0, 0}, rotation: {0, 0, 0, 1},
        //! scale: 1.0)
        virtual const AZ::Transform& GetDefaultSpawnPose() const = 0;

        virtual AZStd::unordered_map<AZStd::string, SpawnPointInfo> GetAllSpawnPointInfos() const = 0;
    };

    class SpawnerNotifications : public AZ::EBusTraits
    {
    public:
        AZ_RTTI(SpawnerNotifications, SpawnerNotificationTypeId);

        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Multiple;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::Single;

        virtual ~SpawnerNotifications() = default;

        //! Notifies that spawn operation was completed.
        //! @param spawnableName name of the spawnable that was spawned.
        //! @param rootEntity root entity of the spawned spawnable.
        //! @param ticketName name of the ticket that was used to spawn the entity.
        virtual void OnSpawned(const AZStd::string& spawnableName, const AZ::EntityId& rootEntity, const AZStd::string& ticketName){};

        //! Notifies that despawn operation was completed.
        //! @param ticketName name of the ticket that was used to despawn the entity.
        virtual void OnDespawned(const AZStd::string& ticketName){};
    };

    using SpawnerRequestsBus = AZ::EBus<SpawnerRequests>;
    using SpawnerNotificationBus = AZ::EBus<SpawnerNotifications>;
} // namespace ROS2
