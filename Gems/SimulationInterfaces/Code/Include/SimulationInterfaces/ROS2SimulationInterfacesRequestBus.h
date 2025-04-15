/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include "ROS2SimulationInterfacesTypeIds.h"

#include <AzCore/EBus/EBus.h>
#include <AzCore/Interface/Interface.h>
#include <AzCore/std/containers/unordered_set.h>

namespace ROS2SimulationInterfaces
{
    class ROS2SimulationInterfacesRequests
    {
    public:
        AZ_RTTI(ROS2SimulationInterfacesRequests, ROS2SimulationInterfacesRequestBusTypeId);
        virtual ~ROS2SimulationInterfacesRequests() = default;

        //! Returns set of simulation features available in ROS2SimulationInterfaces Gem
        //! SimulationFeatureType follows definition available at:
        //! @see https://github.com/ros-simulation/simulation_interfaces/blob/main/msg/SimulatorFeatures.msg
        virtual AZStd::unordered_set<AZ::u8> GetSimulationFeatures() = 0;
    };

    class ROS2SimulationInterfacesRequestBusTraits : public AZ::EBusTraits
    {
    public:
        //////////////////////////////////////////////////////////////////////////
        // EBusTraits overrides
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::Single;
        //////////////////////////////////////////////////////////////////////////
    };

    using ROS2SimulationInterfacesRequestBus = AZ::EBus<ROS2SimulationInterfacesRequests, ROS2SimulationInterfacesRequestBusTraits>;
    using ROS2SimulationInterfacesRequestBusInterface = AZ::Interface<ROS2SimulationInterfacesRequests>;

} // namespace ROS2SimulationInterfaces
