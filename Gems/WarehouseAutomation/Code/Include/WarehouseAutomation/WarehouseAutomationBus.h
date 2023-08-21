/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <WarehouseAutomation/WarehouseAutomationTypeIds.h>

#include <AzCore/EBus/EBus.h>
#include <AzCore/Interface/Interface.h>

namespace WarehouseAutomation
{
    class WarehouseAutomationRequests
    {
    public:
        AZ_RTTI(WarehouseAutomationRequests, WarehouseAutomationRequestsTypeId);
        virtual ~WarehouseAutomationRequests() = default;
        // Put your public methods here
    };

    class WarehouseAutomationBusTraits
        : public AZ::EBusTraits
    {
    public:
        //////////////////////////////////////////////////////////////////////////
        // EBusTraits overrides
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::Single;
        //////////////////////////////////////////////////////////////////////////
    };

    using WarehouseAutomationRequestBus = AZ::EBus<WarehouseAutomationRequests, WarehouseAutomationBusTraits>;
    using WarehouseAutomationInterface = AZ::Interface<WarehouseAutomationRequests>;

} // namespace WarehouseAutomation
