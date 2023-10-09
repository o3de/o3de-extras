/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/RTTI/BehaviorContext.h>
#include <AzCore/RTTI/BehaviorInterfaceProxy.h>

#include <WarehouseAutomation/ProximitySensor/ProximitySensorNotificationBus.h>

namespace WarehouseAutomation
{
    class ProximitySensorNotificationBusHandler
        : public ProximitySensorNotificationBus::Handler
        , public AZ::BehaviorEBusHandler
    {
    public:
        AZ_EBUS_BEHAVIOR_BINDER(
            ProximitySensorNotificationBusHandler,
            "{cc9c2e5a-318d-4212-abeb-95bd0575452d}",
            AZ::SystemAllocator,
            OnObjectInRange,
            OnObjectOutOfRange);

        // Sent when the object is detected.
        void OnObjectInRange() override
        {
            Call(FN_OnObjectInRange);
        }

        // Sent when there are no objects detected.
        void OnObjectOutOfRange() override
        {
            Call(FN_OnObjectOutOfRange);
        }
    };
} // namespace WarehouseAutomation
