/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <SimulationInterfaces/SimulationInterfacesROS2TypeIds.h>

#include <AzCore/EBus/EBus.h>
#include <AzCore/Interface/Interface.h>
#include <AzCore/std/containers/unordered_set.h>

namespace SimulationInterfacesROS2
{
    class SimulationInterfacesROS2Requests
    {
    public:
        AZ_RTTI(SimulationInterfacesROS2Requests, SimulationInterfacesROS2RequestBusTypeId);
        virtual ~SimulationInterfacesROS2Requests() = default;

        //! Returns set of simulation features available in SimulationInterfacesROS2 Gem
        //! SimulationFeatures follows definition available at:
        //! @see https://github.com/ros-simulation/simulation_interfaces/blob/main/msg/SimulatorFeatures.msg
        virtual AZStd::unordered_set<AZ::u8> GetSimulationFeatures() = 0;
    };

    class SimulationInterfacesROS2RequestBusTraits : public AZ::EBusTraits
    {
    public:
        //////////////////////////////////////////////////////////////////////////
        // EBusTraits overrides
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::Single;
        //////////////////////////////////////////////////////////////////////////
    };

    using SimulationInterfacesROS2RequestBus = AZ::EBus<SimulationInterfacesROS2Requests, SimulationInterfacesROS2RequestBusTraits>;
    using SimulationInterfacesROS2RequestBusInterface = AZ::Interface<SimulationInterfacesROS2Requests>;

} // namespace SimulationInterfacesROS2
