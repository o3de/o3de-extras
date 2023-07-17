/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Component/EntityId.h>
#include <AzCore/EBus/EBus.h>

namespace ROS2
{
    //! Interface class that allows to add post-processing to the pipeline
    class ProximitySensorNotifications : public AZ::EBusTraits
    {
    public:
        using BusIdType = AZ::EntityId;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::ById;
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;

        //! Notify that a particular sensor is detecting an object (notification published for each frequency tick).
        virtual void OnObjectInRange() = 0;

        //! Notify that a particular sensor is not detecting an object (notification published for each frequency tick).
        virtual void OnObjectOutOfRange() = 0;

    protected:
        ~ProximitySensorNotifications() = default;
    };

    using ProximitySensorNotificationBus = AZ::EBus<ProximitySensorNotifications>;
} // namespace ROS2
