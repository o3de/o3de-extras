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
#include <AzCore/std/containers/unordered_set.h>
#include <AzCore/std/utility/pair.h>
#include <AzFramework/Entity/EntityContextBus.h>
#include <AzFramework/Input/Devices/Keyboard/InputDeviceKeyboard.h>
#include <AzFramework/Input/Events/InputChannelEventListener.h>
#include <AzFramework/Physics/PhysicsScene.h>
#include <AzFramework/Spawnable/SpawnableEntitiesInterface.h>
#include <SimulationInterfaces/SimulationEntityManagerRequestBus.h>
#include <SimulationInterfaces/SimulationMangerRequestBus.h>
#include <simulation_interfaces/msg/simulation_state.hpp>
#include <simulation_interfaces/srv/set_simulation_state.hpp>

namespace SimulationInterfaces
{
    struct KeyboardTransition
    {
        AzFramework::InputChannelId m_inputChannelId; //! Input channel ID for the keyboard key that triggers the transition
        SimulationState m_desiredState; //! Desired state to transition to when the key is pressed
        AZStd::string m_uiDescription; //! Description of the transition, used in UI
    };
    class SimulationManager
        : public AZ::Component
        , protected SimulationManagerRequestBus::Handler
        , protected AZ::TickBus::Handler
        , protected AzFramework::InputChannelEventListener
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
        AZ::Outcome<void, FailedResult> ResetSimulation(ReloadLevelCallback completionCallback) override;
        void SetSimulationPaused(bool paused) override;
        void StepSimulation(AZ::u64 steps) override;
        bool IsSimulationPaused() const override;
        void CancelStepSimulation() override;
        bool IsSimulationStepsActive() const override;
        SimulationState GetSimulationState() const override;
        AZ::Outcome<void, FailedResult> SetSimulationState(SimulationState stateToSet) override;
        bool EntitiesOperationsPossible() override;

        // AZ::TickBus::Handler
        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;

        // InputChannelEventListener
        bool OnInputChannelEventFiltered(const AzFramework::InputChannel& inputChannel) override;

        //! Register a keyboard transition key in m_keyboardTransitions. Generate UI hints
        //! \param key The input channel ID of the keyboard key
        //! \param sourceState The current simulation state that the key is associated with
        //! \param desiredState The desired simulation state to transition to when the key is pressed
        void RegisterTransitionsKey(const AzFramework::InputChannelId& key, SimulationState sourceState, SimulationState desiredState);

        //! Remove all registered keyboard transition keys
        void UnregisterAllTransitionKeys();

        bool m_isSimulationPaused = false;
        AZStd::optional<AZStd::string> m_levelLoadedAtStartup;

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
        const AZStd::array<AZStd::pair<SimulationState, SimulationState>, 7> m_forbiddenStatesTransitions{
            { { simulation_interfaces::msg::SimulationState::STATE_STOPPED, simulation_interfaces::msg::SimulationState::STATE_PAUSED },
              { simulation_interfaces::msg::SimulationState::STATE_QUITTING, simulation_interfaces::msg::SimulationState::STATE_STOPPED },
              { simulation_interfaces::msg::SimulationState::STATE_QUITTING, simulation_interfaces::msg::SimulationState::STATE_PLAYING },
              { simulation_interfaces::msg::SimulationState::STATE_QUITTING, simulation_interfaces::msg::SimulationState::STATE_PAUSED },
              { simulation_interfaces::msg::SimulationState::STATE_NO_WORLD, simulation_interfaces::msg::SimulationState::STATE_STOPPED },
              { simulation_interfaces::msg::SimulationState::STATE_NO_WORLD, simulation_interfaces::msg::SimulationState::STATE_PLAYING },
              { simulation_interfaces::msg::SimulationState::STATE_NO_WORLD, simulation_interfaces::msg::SimulationState::STATE_PAUSED } }
        };

        //! Map of keyboard transitions - defined in registry key
        AZStd::unordered_map<SimulationState, KeyboardTransition> m_keyboardTransitions;
        bool m_levelLoaded = false;
    };
} // namespace SimulationInterfaces
