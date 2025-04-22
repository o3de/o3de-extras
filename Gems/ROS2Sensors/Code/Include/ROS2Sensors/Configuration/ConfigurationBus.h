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

namespace ROS2 {
    template<typename Configuration>
    class ConfigurationRequests : public AZ::ComponentBus
    {
    public:
        using BusIdType = AZ::EntityId;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::ById;
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Multiple;

        virtual ~ConfigurationRequests() = default;

        virtual const Configuration GetConfiguration() const  {
            return Configuration{};
        };

        virtual void SetConfiguration(const Configuration configuration) {};
    };

    template<typename Configuration>
    using ConfigurationBus = AZ::EBus<ConfigurationRequests<Configuration>>;
}
