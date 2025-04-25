/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include "SimulationInterfacesTypeIds.h"

#include "Result.h"
#include <AzCore/EBus/EBus.h>
#include <AzCore/Interface/Interface.h>
#include <AzCore/std/smart_ptr/shared_ptr.h>
#include <AzFramework/Physics/ShapeConfiguration.h>
#include <simulation_interfaces/msg/simulation_state.hpp>

namespace SimulationInterfaces
{
    using SimulationState = simulation_interfaces::msg::SimulationState::_state_type;

    class SimulationManagerRequests
    {
    public:
        AZ_RTTI(SimulationManagerRequests, SimulationManagerRequestsTypeId);
        virtual ~SimulationManagerRequests() = default;

        using ReloadLevelCallback = AZStd::function<void(void)>;

        //! Reload level and remove all spawned simulation entities.
        virtual void ReloadLevel(ReloadLevelCallback completionCallback) = 0;

        //! Set the simulation to paused or unpaused,
        //! expect always to succeed
        virtual void SetSimulationPaused(bool paused) = 0;

        //! Step the simulation by a number of steps
        //! expect always to succeed
        virtual void StepSimulation(AZ::u64 steps) = 0;

        //! Check whether the simulation is paused or not
        //! @return boolean value indicating the pause state of the simulation
        virtual bool IsSimulationPaused() const = 0;

        //! Cancel executing the simulation steps, if there are remaining steps left
        virtual void CancelStepSimulation() = 0;

        //! Check if the SimulationSteps is active
        //! @return boolean value indicating if the SimulationSteps is active
        virtual bool IsSimulationStepsActive() const = 0;

        //! Get current simulation state
        //! @return enum value indicating simulation state based on
        //! https://github.com/ros-simulation/simulation_interfaces/blob/main/msg/SimulationState.msg
        virtual SimulationState GetSimulationState() const = 0;

        //! Set simulation state
        //! @param stateToSet id of the next state to set, based on
        //! https://github.com/ros-simulation/simulation_interfaces/blob/main/msg/SimulationState.msg
        //! @return outcome indicating if setting state succeed. In case of failure error message with error code is returned
        virtual AZ::Outcome<void, FailedResult> SetSimulationState(SimulationState stateToSet) = 0;
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

    class SimulationManagerNotifications
    {
    public:
        AZ_RTTI(SimulationManagerNotifications, SimulationManagerNotificationsTypeId);
        virtual ~SimulationManagerNotifications() = default;

        //! Notify about simulation step finish
        //! @param remainingSteps - remaining steps to pause simulation
        virtual void OnSimulationStepFinish(const AZ::u64 remainingSteps) = 0;
    };

    class SimulationMangerNotificationsBusTraits : public AZ::EBusTraits
    {
    public:
        //////////////////////////////////////////////////////////////////////////
        // EBusTraits overrides
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Multiple;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::Single;
        //////////////////////////////////////////////////////////////////////////
    };

    using SimulationManagerNotificationsBus = AZ::EBus<SimulationManagerNotifications, SimulationMangerNotificationsBusTraits>;

} // namespace SimulationInterfaces
