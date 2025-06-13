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
#include <AzCore/RTTI/BehaviorContext.h>
#include <ROS2/ROS2TypeIds.h>
#include <ROS2/Spawner/SpawnerBus.h>

namespace ROS2
{
    class SpawnerNotificationsBusHandler
        : public SpawnerNotificationBus::Handler
        , public AZ::BehaviorEBusHandler
    {
    public:
        AZ_EBUS_BEHAVIOR_BINDER(SpawnerNotificationsBusHandler, SpawnerBusHandlerTypeId, AZ::SystemAllocator, OnSpawned, OnDespawned);

        void OnSpawned(const AZStd::string& spawnableName, const AZ::EntityId& rootEntity, const AZStd::string& ticketName) override
        {
            Call(FN_OnSpawned, spawnableName, rootEntity, ticketName);
        }

        void OnDespawned(const AZStd::string& ticketName) override
        {
            Call(FN_OnDespawned, ticketName);
        }
    };

} // namespace ROS2
