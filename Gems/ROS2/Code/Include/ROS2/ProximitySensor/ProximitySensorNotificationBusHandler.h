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

#include <ROS2/ProximitySensor/ProximitySensorNotificationBus.h>

namespace ROS2
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
            OnObjectDetected,
            OnObjectUnseen);

        // Sent when the object is detected.
        void OnObjectDetected() override
        {
            Call(FN_OnObjectDetected);
        }

        // Sent when there are no objects detected.
        void OnObjectUnseen() override
        {
            Call(FN_OnObjectUnseen);
        }
    };
} // namespace ROS2
