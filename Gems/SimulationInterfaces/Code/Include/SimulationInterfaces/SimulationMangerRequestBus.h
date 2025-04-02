/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <SimulationInterfaces/SimulationInterfacesTypeIds.h>

#include <AzCore/EBus/EBus.h>
#include <AzCore/Interface/Interface.h>
#include <AzCore/std/smart_ptr/shared_ptr.h>
#include <AzFramework/Physics/ShapeConfiguration.h>

namespace SimulationInterfaces
{

    class SimulationManagerRequests
    {
    public:
        AZ_RTTI(SimulationManagerRequests, SimulationManagerRequestsTypeId);
        virtual ~SimulationManagerRequests() = default;

        virtual void SetSimulationPaused(bool paused) = 0;
        virtual void StepSimulation(AZ::u32 steps) = 0;

    };

    class SimulationMangerRequestBusTraits : public AZ::EBusTraits
    {
    public:
        //////////////////////////////////////////////////////////////////////////
        // EBusTraits overrides
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::Single;
        //////////////////////////////////////////////////////////////////////////
    };

    using SimulationManagerRequestBus = AZ::EBus<SimulationManagerRequests, SimulationMangerRequestBusTraits>;
    using SimulationManagerRequestBusInterface = AZ::Interface<SimulationManagerRequests>;

} // namespace SimulationInterfaces
