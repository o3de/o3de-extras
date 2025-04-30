/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "SimulationManager.h"
#include "SimulationInterfaces/SimulationMangerRequestBus.h"

#include <AzCore/Component/ComponentApplicationBus.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/Settings/SettingsRegistry.h>
#include <AzFramework/Components/ConsoleBus.h>
#include <AzFramework/Physics/PhysicsSystem.h>
#include <DebugDraw/DebugDrawBus.h>
#include <SimulationInterfaces/SimulationFeaturesAggregatorRequestBus.h>
#include <SimulationInterfaces/SimulationInterfacesTypeIds.h>
#include <simulation_interfaces/msg/simulator_features.hpp>

namespace SimulationInterfaces
{
    namespace
    {

        const AZStd::unordered_map<SimulationState, AZStd::string> SimulationStateToString = {
            { simulation_interfaces::msg::SimulationState::STATE_PAUSED, "STATE_PAUSED" },
            { simulation_interfaces::msg::SimulationState::STATE_PLAYING, "STATE_PLAYING" },
            { simulation_interfaces::msg::SimulationState::STATE_QUITTING, "STATE_QUITTING" },
            { simulation_interfaces::msg::SimulationState::STATE_STOPPED, "STATE_STOPPED" }
        };

        constexpr AZStd::string_view PrintStateName = "/SimulationInterfaces/PrintStateNameInGui";
        constexpr AZStd::string_view StartInStoppedStateKey = "/SimulationInterfaces/StartInStoppedState";

        AZStd::string GetStateName(SimulationState state)
        {
            auto it = SimulationStateToString.find(state);
            if (it != SimulationStateToString.end())
            {
                return it->second;
            }
            return AZStd::string::format("Unknown state: %d", static_cast<int>(state));
        }

        bool StartInStoppedState()
        {
            AZ::SettingsRegistryInterface* settingsRegistry = AZ::SettingsRegistry::Get();
            AZ_Assert(settingsRegistry, "Settings Registry is not available");
            bool output = true;
            settingsRegistry->Get(output, StartInStoppedStateKey);
            return output;
        }

        bool PrintStateNameInGui()
        {
            AZ::SettingsRegistryInterface* settingsRegistry = AZ::SettingsRegistry::Get();
            AZ_Assert(settingsRegistry, "Settings Registry is not available");
            bool output = true;
            settingsRegistry->Get(output, PrintStateName);
            return output;
        }
    } // namespace

    AZ_COMPONENT_IMPL(SimulationManager, "SimulationManager", SimulationManagerTypeId);

    void SimulationManager::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<SimulationManager, AZ::Component>()->Version(0);
        }
    }

    void SimulationManager::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("SimulationManagerService"));
    }

    void SimulationManager::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("SimulationManagerService"));
    }

    void SimulationManager::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("PhysicsService"));
        required.push_back(AZ_CRC_CE("SimulationFeaturesAggregator"));
    }

    void SimulationManager::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
        dependent.push_back(AZ_CRC_CE("SimulationFeaturesAggregator"));
        dependent.push_back(AZ_CRC_CE("DebugDrawTextService"));
    }

    SimulationManager::SimulationManager()
    {
        if (SimulationManagerRequestBusInterface::Get() == nullptr)
        {
            SimulationManagerRequestBusInterface::Register(this);
        }
    }

    SimulationManager::~SimulationManager()
    {
        if (SimulationManagerRequestBusInterface::Get() == this)
        {
            SimulationManagerRequestBusInterface::Unregister(this);
        }
    }

    void SimulationManager::InitializeSimulationState()
    {
        // if start in stopped state, pause simulation. Default state for simulation by the standard is STOPPED and
        // SetSimulationState has logic to prevent transition to the same state.
        if (StartInStoppedState())
        {
            m_simulationState = simulation_interfaces::msg::SimulationState::STATE_STOPPED;
            SetSimulationPaused(true);
        }
        else
        {
            SetSimulationState(simulation_interfaces::msg::SimulationState::STATE_PLAYING);
        }
    }

    void SimulationManager::Activate()
    {
        AzFramework::LevelSystemLifecycleNotificationBus::Handler::BusDisconnect();
        SimulationManagerRequestBus::Handler::BusConnect();
        SimulationFeaturesAggregatorRequestBus::Broadcast(
            &SimulationFeaturesAggregatorRequests::AddSimulationFeatures,
            AZStd::unordered_set<SimulationFeatureType>{ simulation_interfaces::msg::SimulatorFeatures::SIMULATION_RESET,
                                                         simulation_interfaces::msg::SimulatorFeatures::SIMULATION_RESET_TIME,
                                                         simulation_interfaces::msg::SimulatorFeatures::SIMULATION_RESET_STATE,
                                                         simulation_interfaces::msg::SimulatorFeatures::SIMULATION_RESET_SPAWNED,
                                                         simulation_interfaces::msg::SimulatorFeatures::SIMULATION_STATE_PAUSE,
                                                         simulation_interfaces::msg::SimulatorFeatures::STEP_SIMULATION_SINGLE,
                                                         simulation_interfaces::msg::SimulatorFeatures::STEP_SIMULATION_MULTIPLE,
                                                         simulation_interfaces::msg::SimulatorFeatures::STEP_SIMULATION_ACTION,
                                                         simulation_interfaces::msg::SimulatorFeatures::SIMULATION_STATE_SETTING,
                                                         simulation_interfaces::msg::SimulatorFeatures::SIMULATION_STATE_GETTING });
        if (PrintStateNameInGui())
        {
            AZ::TickBus::Handler::BusConnect();
        }
        AZ::SystemTickBus::QueueFunction(
            [this]()
            {
                InitializeSimulationState();
            });
    }

    void SimulationManager::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
        SimulationManagerRequestBus::Handler::BusDisconnect();
    }

    bool SimulationManager::IsSimulationPaused() const
    {
        return m_isSimulationPaused;
    }

    bool SimulationManager::IsSimulationStepsActive() const
    {
        return m_simulationFinishEvent.IsConnected();
    }

    void SimulationManager::CancelStepSimulation()
    {
        if (m_simulationFinishEvent.IsConnected())
        {
            m_simulationFinishEvent.Disconnect();
            SetSimulationPaused(true);
            m_numberOfPhysicsSteps = 0;
        }
    }

    void SimulationManager::SetSimulationPaused(bool paused)
    {
        // get az physics system
        auto* physicsSystem = AZ::Interface<AzPhysics::SystemInterface>::Get();
        AZ_Assert(physicsSystem, "Physics system is not available");
        const auto& sceneHandlers = physicsSystem->GetAllScenes();
        [[maybe_unused]] auto* sceneInterface = AZ::Interface<AzPhysics::SceneInterface>::Get();
        AZ_Assert(sceneInterface, "Physics scene interface is not available");
        for (auto& scene : sceneHandlers)
        {
            AZ_Assert(scene, "Physics scene is not available");
            scene->SetEnabled(!paused);
            m_isSimulationPaused = paused;
        }
    }

    void SimulationManager::StepSimulation(AZ::u64 steps)
    {
        if (steps == 0)
        {
            return;
        }
        m_numberOfPhysicsSteps = steps;

        // install handler
        m_simulationFinishEvent = AzPhysics::SceneEvents::OnSceneSimulationFinishHandler(
            [this](AzPhysics::SceneHandle sceneHandle, float)
            {
                m_numberOfPhysicsSteps--;
                AZ_Printf("SimulationManager", "Physics simulation step finished. Remaining steps: %d", m_numberOfPhysicsSteps);
                SimulationManagerNotificationsBus::Broadcast(
                    &SimulationManagerNotifications::OnSimulationStepFinish, m_numberOfPhysicsSteps);
                if (m_numberOfPhysicsSteps <= 0)
                {
                    SetSimulationPaused(true);
                    // remove handler
                    m_simulationFinishEvent.Disconnect();
                }
            });

        // get default scene
        [[maybe_unused]] auto* physicsSystem = AZ::Interface<AzPhysics::SystemInterface>::Get();
        AZ_Assert(physicsSystem, "Physics system is not available");
        auto* sceneInterface = AZ::Interface<AzPhysics::SceneInterface>::Get();
        AZ_Assert(sceneInterface, "Physics scene interface is not available");
        AzPhysics::SceneHandle defaultScene = sceneInterface->GetSceneHandle(AzPhysics::DefaultPhysicsSceneName);

        auto scene = sceneInterface->GetScene(defaultScene);
        AZ_Assert(scene, "Default physics scene is not available");

        // install handler
        scene->RegisterSceneSimulationFinishHandler(m_simulationFinishEvent);
        SetSimulationPaused(false);
    }

    void SimulationManager::ReloadLevel(SimulationManagerRequests::ReloadLevelCallback completionCallback)
    {
        AzFramework::LevelSystemLifecycleNotificationBus::Handler::BusConnect();
        m_reloadLevelCallback = completionCallback;

        // We need to delete all entities before reloading the level
        DeletionCompletedCb deleteAllCompletion = [](const AZ::Outcome<void, FailedResult>& result)
        {
            AZ_Trace("SimulationManager", "Delete all entities completed: %s, reload level", result.IsSuccess() ? "true" : "false");
            const char* levelName = AZ::Interface<AzFramework::ILevelSystemLifecycle>::Get()->GetCurrentLevelName();
            AzFramework::ConsoleRequestBus::Broadcast(&AzFramework::ConsoleRequests::ExecuteConsoleCommand, "UnloadLevel");
            AZStd::string command = AZStd::string::format("LoadLevel %s", levelName);
            AzFramework::ConsoleRequestBus::Broadcast(&AzFramework::ConsoleRequests::ExecuteConsoleCommand, command.c_str());
        };

        // delete spawned entities
        SimulationEntityManagerRequestBus::Broadcast(&SimulationEntityManagerRequests::DeleteAllEntities, deleteAllCompletion);
    }

    void SimulationManager::OnLoadingComplete(const char* levelName)
    {
        AZ_Printf("SimulationManager", "Level loading started: %s", levelName);
        if (m_reloadLevelCallback)
        {
            m_reloadLevelCallback();
            m_reloadLevelCallback = nullptr;
        }
        // reset of the simulation, assign the same state as at the beginning
        InitializeSimulationState();
        AzFramework::LevelSystemLifecycleNotificationBus::Handler::BusDisconnect();
    }

    SimulationState SimulationManager::GetSimulationState() const
    {
        return m_simulationState;
    }

    AZ::Outcome<void, FailedResult> SimulationManager::SetSimulationState(SimulationState stateToSet)
    {
        // check if simulation is in desire state
        if (m_simulationState == stateToSet)
        {
            return AZ::Failure(FailedResult(
                simulation_interfaces::srv::SetSimulationState::Response::ALREADY_IN_TARGET_STATE,
                "Simulation is already in requested state, transition unecessary"));
        }

        if (IsTransitionForbiddenInEditor(stateToSet))
        {
            const auto stateToSetName = GetStateName(stateToSet);
            const auto currentStateName = GetStateName(m_simulationState);
            return AZ::Failure(FailedResult(
                simulation_interfaces::srv::SetSimulationState::Response::INCORRECT_TRANSITION,
                AZStd::string::format(
                    "Requested transition (%s -> %s) is forbidden in the Editor. It is available in GameLauncher.",
                    currentStateName.c_str(),
                    stateToSetName.c_str())));
        }
        if (IsTransitionForbidden(stateToSet))
        {
            const auto stateToSetName = GetStateName(stateToSet);
            const auto currentStateName = GetStateName(m_simulationState);
            return AZ::Failure(FailedResult(
                simulation_interfaces::srv::SetSimulationState::Response::INCORRECT_TRANSITION,
                AZStd::string::format("Requested transition (%s -> %s) is forbidden", currentStateName.c_str(), stateToSetName.c_str())));
        }

        switch (stateToSet)
        {
        case simulation_interfaces::msg::SimulationState::STATE_STOPPED:
            {
                SimulationManagerRequests::ReloadLevelCallback cb = []()
                {
                    SimulationInterfaces::SimulationManagerRequestBus::Broadcast(
                        &SimulationInterfaces::SimulationManagerRequests::SetSimulationPaused, true);
                };
                ReloadLevel(cb);
                break;
            }
        case simulation_interfaces::msg::SimulationState::STATE_PLAYING:
            {
                SetSimulationPaused(false);
                break;
            }
        case simulation_interfaces::msg::SimulationState::STATE_PAUSED:
            {
                SetSimulationPaused(true);
                break;
            }
        case simulation_interfaces::msg::SimulationState::STATE_QUITTING:
            {
                // stop simulation -> kill the simulator.
                SetSimulationPaused(true);

                // queue to allow status of this method to be returned, then start quitting
                AZ::SystemTickBus::QueueFunction(
                    []()
                    {
                        AzFramework::ConsoleRequestBus::Broadcast(&AzFramework::ConsoleRequests::ExecuteConsoleCommand, "quit");
                    });
                break;
            }
        default:
            {
                return AZ::Failure(FailedResult(
                    simulation_interfaces::srv::SetSimulationState::Response::INCORRECT_TRANSITION, "Requested state doesn't exists"));
                break;
            }
        }
        m_simulationState = stateToSet;
        return AZ::Success();
    }

    bool SimulationManager::IsTransitionForbiddenInEditor(SimulationState requestedState)
    {
        // in the Editor we cannot reload level, so going to STOPPED state is forbidden, we cannot quit the editor so going to QUITTING
        // state is forbidden
        AZ::ApplicationTypeQuery appType;
        AZ::ComponentApplicationBus::Broadcast(&AZ::ComponentApplicationBus::Events::QueryApplicationType, appType);
        if (appType.IsValid() && !appType.IsGame())
        {
            if (requestedState == simulation_interfaces::msg::SimulationState::STATE_STOPPED ||
                requestedState == simulation_interfaces::msg::SimulationState::STATE_QUITTING)
            {
                return true;
            }
        }
        return false;
    }

    bool SimulationManager::IsTransitionForbidden(SimulationState requestedState)
    {
        AZStd::pair<SimulationState, SimulationState> desireTransition{ m_simulationState, requestedState };
        auto it = AZStd::find(m_forbiddenStatesTransitions.begin(), m_forbiddenStatesTransitions.end(), desireTransition);
        return it != m_forbiddenStatesTransitions.end();
    }

    void SimulationManager::OnTick([[maybe_unused]] float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
        DebugDraw::DebugDrawRequestBus::Broadcast(
            &DebugDraw::DebugDrawRequests::DrawTextOnScreen,
            AZStd::string::format("Simulation state: %s", GetStateName(m_simulationState).c_str()),
            AZ::Color(1.0f, 1.0f, 1.0f, 1.0f),
            0.f);
    }

} // namespace SimulationInterfaces
