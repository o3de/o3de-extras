/*
 * Copyright (c) Contributors to the Open 3D Engine Project
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/EBus/EBus.h>
#include <Source/${SanitizedCppName}Types.h>

namespace ${SanitizedCppName}
{
    class WeaponNotifications
        : public AZ::EBusTraits
    {
    public:
        virtual ~WeaponNotifications() = default;

        //! Called on a local player that has activated a weapon.
        virtual void OnWeaponActivate([[maybe_unused]] AZ::EntityId shooterEntityId, [[maybe_unused]] const AZ::Transform& transform) {}

        //! Called on a local player that has predictively impacted something with a weapon.
        virtual void OnWeaponImpact([[maybe_unused]] AZ::EntityId shooterEntityId, [[maybe_unused]] const AZ::Transform& transform, [[maybe_unused]] AZ::EntityId hitEntityId) {}

        //! Called on a local player that has confirmed damaged something with a weapon.
        virtual void OnWeaponDamage([[maybe_unused]] AZ::EntityId shooterEntityId, [[maybe_unused]] const AZ::Transform& transform, [[maybe_unused]] AZ::EntityId hitEntityId) {}

        //! Called on a local player that has been confirmed to hit a player with a weapon.
        virtual void OnConfirmedHitPlayer([[maybe_unused]] AZ::EntityId byPlayerEntity, [[maybe_unused]] AZ::EntityId otherPlayerEntity) {}
    };

    using WeaponNotificationBus = AZ::EBus<WeaponNotifications>;
}
