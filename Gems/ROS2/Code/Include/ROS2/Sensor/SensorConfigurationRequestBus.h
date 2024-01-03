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
#include <AzCore/Interface/Interface.h>
#include <ROS2/Sensor/SensorConfiguration.h>

namespace ROS2
{
    //! Interface that allows to get sensor configuration and switch publish state.
    class SensorConfigurationRequest : public AZ::EBusTraits
    {
    public:
        using BusIdType = AZ::EntityId;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::ById;

        virtual SensorConfiguration GetSensorConfiguration() const = 0;
        virtual void EnablePublishing(bool publishingEnabled) = 0;
    };

    using SensorConfigurationRequestBus = AZ::EBus<SensorConfigurationRequest>;
} // namespace ROS2
