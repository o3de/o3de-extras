/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <SimulationInterfaces/SimulationInterfacesTypeIds.h>

#include "SimulationFeatures.h"
#include <AzCore/EBus/EBus.h>
#include <AzCore/Interface/Interface.h>
#include <AzCore/std/containers/unordered_set.h>
#include <AzCore/std/smart_ptr/shared_ptr.h>
#include <AzFramework/Physics/ShapeConfiguration.h>

namespace SimulationInterfaces
{

    class SimulationFeaturesAggregatorRequests
    {
    public:
        AZ_RTTI(SimulationFeaturesAggregatorRequests, SimulationFeaturesAggregatorRequestsTypeId);
        virtual ~SimulationFeaturesAggregatorRequests() = default;

        //! Registers simulation features defined by caller
        virtual void AddSimulationFeatures(AZStd::unordered_set<SimulationFeatures> features) = 0;

        //! Returns features available in the simulator, list follows definitions at
        //! @see https://github.com/ros-simulation/simulation_interfaces/blob/main/msg/SimulatorFeatures.msg
        virtual AZStd::unordered_set<SimulationFeatures> GetSimulationFeatures() = 0;
    };

    class SimulationFeaturesAggregatorRequestBusTraits : public AZ::EBusTraits
    {
    public:
        //////////////////////////////////////////////////////////////////////////
        // EBusTraits overrides
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::Single;
        //////////////////////////////////////////////////////////////////////////
    };

    using SimulationFeaturesAggregatorRequestBus =
        AZ::EBus<SimulationFeaturesAggregatorRequests, SimulationFeaturesAggregatorRequestBusTraits>;
    using SimulationFeaturesAggregatorRequestBusInterface = AZ::Interface<SimulationFeaturesAggregatorRequests>;

} // namespace SimulationInterfaces
