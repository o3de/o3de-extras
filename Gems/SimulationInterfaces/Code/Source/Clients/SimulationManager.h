/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/Component/Component.h>
#include <AzCore/Component/TickBus.h>
#include <AzCore/Script/ScriptTimePoint.h>
#include <AzCore/std/containers/array.h>
#include <AzCore/std/utility/pair.h>
#include <AzFramework/API/ApplicationAPI.h>
#include <AzFramework/Entity/EntityContextBus.h>
#include <AzFramework/Physics/PhysicsScene.h>
#include <AzFramework/Spawnable/SpawnableEntitiesInterface.h>
#include <SimulationInterfaces/SimulationEntityManagerRequestBus.h>
#include <SimulationInterfaces/SimulationMangerRequestBus.h>
#include <simulation_interfaces/srv/set_simulation_state.hpp>

namespace SimulationInterfaces
{
    class SimulationManager
        : public AZ::Component
        , protected SimulationManagerRequestBus::Handler
        , protected AzFramework::LevelSystemLifecycleNotificationBus::Handler
        , protected AZ::TickBus::Handler
    {
    public:
        AZ_COMPONENT_DECL(SimulationManager);

        static void Reflect(AZ::ReflectContext* context);

        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);

        SimulationManager();
        ~SimulationManager();

        // AZ::Component
        void Activate() override;
        void Deactivate() override;

    private:
        // SimulationManagerRequestBus interface implementation
        void SetSimulationPaused(bool paused) override;
        void StepSimulation(AZ::u64 steps) override;
        bool IsSimulationPaused() const override;
        void CancelStepSimulation() override;
        bool IsSimulationStepsActive() const override;
        void ReloadLevel(SimulationManagerRequests::ReloadLevelCallback completionCallback) override;
        SimulationState GetSimulationState() const override;
        AZ::Outcome<void, FailedResult> SetSimulationState(SimulationState stateToSet) override;

        // LevelSystemLifecycleNotificationBus interface implementation
        void OnLoadingComplete(const char* levelName) override;

        // AZ::TickBus::Handler
        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;

        bool m_isSimulationPaused = false;

        uint64_t m_numberOfPhysicsSteps = 0;
        AzPhysics::SceneEvents::OnSceneSimulationFinishHandler m_simulationFinishEvent;
        SimulationManagerRequests::ReloadLevelCallback m_reloadLevelCallback;
        SimulationState m_simulationState{
            simulation_interfaces::msg::SimulationState::STATE_STOPPED
        }; // default simulation state based on standard
        void InitializeSimulationState();

        bool IsTransitionForbiddenInEditor(SimulationState requestedState);

        bool IsTransitionForbidden(SimulationState requestedState);
        // forbidden transition between state, first is current state, second is desire state
        const AZStd::array<AZStd::pair<SimulationState, SimulationState>, 4> m_forbiddenStatesTransitions{ {
            { simulation_interfaces::msg::SimulationState::STATE_STOPPED, simulation_interfaces::msg::SimulationState::STATE_PAUSED },
            { simulation_interfaces::msg::SimulationState::STATE_QUITTING, simulation_interfaces::msg::SimulationState::STATE_STOPPED },
            { simulation_interfaces::msg::SimulationState::STATE_QUITTING, simulation_interfaces::msg::SimulationState::STATE_PLAYING },
            { simulation_interfaces::msg::SimulationState::STATE_QUITTING, simulation_interfaces::msg::SimulationState::STATE_PAUSED },
        } };
    };
} // namespace SimulationInterfaces
