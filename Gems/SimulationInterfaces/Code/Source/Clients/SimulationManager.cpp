/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "SimulationManager.h"
#include <AzCore/Component/ComponentApplicationBus.h>
#include <AzCore/Component/TickBus.h>
#include <AzCore/Outcome/Outcome.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/Settings/SettingsRegistry.h>
#include <AzFramework/Components/ConsoleBus.h>
#include <AzFramework/Physics/PhysicsSystem.h>
#include <DebugDraw/DebugDrawBus.h>
#include <SimulationInterfaces/LevelManagerRequestBus.h>
#include <SimulationInterfaces/SimulationFeaturesAggregatorRequestBus.h>
#include <SimulationInterfaces/SimulationInterfacesTypeIds.h>
#include <SimulationInterfaces/SimulationMangerRequestBus.h>
#include <SimulationInterfaces/WorldResource.h>
#include <simulation_interfaces/msg/result.hpp>
#include <simulation_interfaces/msg/simulation_state.hpp>
#include <simulation_interfaces/msg/simulator_features.hpp>
#include <simulation_interfaces/srv/get_current_world.hpp>

namespace SimulationInterfaces
{
    namespace
    {

        //! Convert string like : keyboard_key_alphanumeric_O to a pretty name like "Key 'O'"
        AZStd::string MakePrettyKeyboardName(const AzFramework::InputChannelId& inputChannelId)
        {
            // Convert the input channel name to a pretty format for display

            // split the name by underscores
            AZStd::vector<AZStd::string> parts;
            AZ::StringFunc::Tokenize(inputChannelId.GetName(), parts, "_");
            if (parts.size() < 4)
            {
                return inputChannelId.GetName(); // return original if no parts found
            }

            const AZStd::string& deviceType = parts[0];
            const AZStd::string& keyType = parts[1];
            const AZStd::string& keyRegion = parts[2];
            const AZStd::string& keyName = parts[3];

            if (deviceType != "keyboard" || keyType != "key")
            {
                return inputChannelId.GetName(); // return original if not a keyboard alphanumeric key
            }

            // Create a pretty name based on the key region and key name
            if (keyRegion == "alphanumeric" || keyRegion == "function")
            {
                // For alphanumeric keys, we can return the key name directly
                return AZStd::string::format("Key '%s'", keyName.c_str());
            }

            return AZStd::string::format("Key '%s_%s'", keyRegion.c_str(), keyName.c_str());
        }

        const AZStd::unordered_map<SimulationState, AZStd::string> SimulationStateToString = {
            { simulation_interfaces::msg::SimulationState::STATE_PAUSED, "STATE_PAUSED" },
            { simulation_interfaces::msg::SimulationState::STATE_PLAYING, "STATE_PLAYING" },
            { simulation_interfaces::msg::SimulationState::STATE_QUITTING, "STATE_QUITTING" },
            { simulation_interfaces::msg::SimulationState::STATE_STOPPED, "STATE_STOPPED" },
            { simulation_interfaces::msg::SimulationState::STATE_NO_WORLD, "STATE_NO_WORLD" },
            { simulation_interfaces::msg::SimulationState::STATE_LOADING_WORLD, "STATE_LOADING_WORLD" }
        };

        constexpr AZStd::string_view PrintStateName = "/SimulationInterfaces/PrintStateNameInGui";
        constexpr AZStd::string_view StartInStoppedStateKey = "/SimulationInterfaces/StartInStoppedState";
        constexpr AZStd::string_view KeyboardTransitionStoppedToPlaying = "/SimulationInterfaces/KeyboardTransitions/StoppedToPlaying";
        constexpr AZStd::string_view KeyboardTransitionPausedToPlaying = "/SimulationInterfaces/KeyboardTransitions/PausedToPlaying";
        constexpr AZStd::string_view KeyboardTransitionPlayingToPaused = "/SimulationInterfaces/KeyboardTransitions/PlayingToPaused";

        AZStd::string GetStateName(SimulationState state)
        {
            auto it = SimulationStateToString.find(state);
            if (it != SimulationStateToString.end())
            {
                return it->second;
            }
            return AZStd::string::format("Unknown state: %d", aznumeric_cast<int>(state));
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

        AZStd::optional<AzFramework::InputChannelId> GetKeyboardTransitionKey(const AZStd::string& registryKeyName)
        {
            AZ::SettingsRegistryInterface* settingsRegistry = AZ::SettingsRegistry::Get();
            AZ_Assert(settingsRegistry, "Settings Registry is not available");
            AZStd::string channelIdName;
            settingsRegistry->Get(channelIdName, registryKeyName);

            if (channelIdName.empty())
            {
                AZ_Error("SimulationManager", false, "Failed to get keyboard transition key from registry: %s", registryKeyName.c_str());
                return AZStd::nullopt;
            }
            AZ::Crc32 channelIdCrc32 = AZ::Crc32(channelIdName.c_str());

            for (const auto& inputChannel : AzFramework::InputDeviceKeyboard::Key::All)
            {
                if (inputChannel.GetNameCrc32() == channelIdCrc32)
                {
                    return inputChannel;
                }
            }
            AZ_Error("SimulationManager", false, "Failed to find input channel with name: %s", channelIdName.c_str());
            return AZStd::nullopt;
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
        required.push_back(AZ_CRC_CE("LevelManagerService"));
    }

    void SimulationManager::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
        dependent.push_back(AZ_CRC_CE("SimulationFeaturesAggregator"));
        dependent.push_back(AZ_CRC_CE("LevelManagerService"));
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

        // Query registry for keyboard transition keys
        if (const auto stoppedToPlayingKey = GetKeyboardTransitionKey(KeyboardTransitionStoppedToPlaying))
        {
            RegisterTransitionsKey(
                *stoppedToPlayingKey,
                simulation_interfaces::msg::SimulationState::STATE_STOPPED,
                simulation_interfaces::msg::SimulationState::STATE_PLAYING);
        }

        if (const auto pausedToPlayingKey = GetKeyboardTransitionKey(KeyboardTransitionPausedToPlaying))
        {
            RegisterTransitionsKey(
                *pausedToPlayingKey,
                simulation_interfaces::msg::SimulationState::STATE_PAUSED,
                simulation_interfaces::msg::SimulationState::STATE_PLAYING);
        }

        if (const auto playingToPausedKey = GetKeyboardTransitionKey(KeyboardTransitionPlayingToPaused))
        {
            RegisterTransitionsKey(
                *playingToPausedKey,
                simulation_interfaces::msg::SimulationState::STATE_PLAYING,
                simulation_interfaces::msg::SimulationState::STATE_PAUSED);
        }
        InputChannelEventListener::BusConnect();

        AZ::ApplicationTypeQuery appType;
        AZ::ComponentApplicationBus::Broadcast(&AZ::ComponentApplicationBus::Events::QueryApplicationType, appType);

        // wait one tick to allow all system to start to ensure correct bus calls related to setting simulation state
        if (appType.IsEditor())
        {
            // if app is editor, buses for getting current world are inactive and return fail. In editor we need to simply initialize state
            AZ::SystemTickBus::QueueFunction(
                [this]()
                {
                    InitializeSimulationState();
                });
        }
        else if (appType.IsGame())
        {
            AZ::SystemTickBus::QueueFunction(
                [this]()
                {
                    // check if level is loaded
                    AZ::Outcome<WorldResource, FailedResult> getCurrentWorldOutcome;
                    LevelManagerRequestBus::BroadcastResult(getCurrentWorldOutcome, &LevelManagerRequestBus::Events::GetCurrentWorld);

                    if (!getCurrentWorldOutcome.IsSuccess() &&
                        getCurrentWorldOutcome.GetError().m_errorCode ==
                            simulation_interfaces::srv::GetCurrentWorld::Response::NO_WORLD_LOADED)
                    {
                        SetSimulationState(simulation_interfaces::msg::SimulationState::STATE_NO_WORLD);
                    }
                    else if (getCurrentWorldOutcome.IsSuccess())
                    {
                        m_levelLoadedAtStartup = getCurrentWorldOutcome.GetValue().m_worldResource.m_uri;
                        InitializeSimulationState();
                    }
                });
        }
    }

    void SimulationManager::Deactivate()
    {
        InputChannelEventListener::BusDisconnect();
        UnregisterAllTransitionKeys();
        AZ::TickBus::Handler::BusDisconnect();
        SimulationManagerRequestBus::Handler::BusDisconnect();
        m_levelLoadedAtStartup.reset();
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

    AZ::Outcome<void, FailedResult> SimulationManager::ResetSimulation(ReloadLevelCallback completionCallback)
    {
        // reset is possible only if simulation is already started - world is loaded, or simulation interfaces has in cache level loaded
        // during startup. Only in this case reseting makes sense. If there is no info about level loaded at startup and there is no world
        // loaded -> simulation is in the exactly same state as right after startup
        if ((m_simulationState == simulation_interfaces::msg::SimulationState::STATE_LOADING_WORLD ||
             m_simulationState == simulation_interfaces::msg::SimulationState::STATE_NO_WORLD) &&
            !m_levelLoadedAtStartup.has_value())
        {
            return AZ::Failure(FailedResult(
                simulation_interfaces::msg::Result::RESULT_OPERATION_FAILED,
                "Cannot reset simulation without loaded level and without knowledge about level loaded during startup. Simulator is "
                "already in the same state as after start up"));
        }
        m_reloadLevelCallback = completionCallback;
        // We need to delete all entities before reloading the level
        DeletionCompletedCb deleteAllCompletion =
            [levelLoadedAtStartup = m_levelLoadedAtStartup, completionCallback](const AZ::Outcome<void, FailedResult>& result)
        {
            AZ_Info("SimulationManager", "Delete all entities completed: %s, reload level", result.IsSuccess() ? "true" : "false");
            // queue required to allow all resources related to removed spawnables to be released, especially those related to level.pak
            AZ::SystemTickBus::QueueFunction(
                [levelLoadedAtStartup, completionCallback]()
                {
                    // call level manager to reload the level
                    if (levelLoadedAtStartup.has_value())
                    {
                        LoadWorldRequest request;
                        request.levelResource.m_uri = levelLoadedAtStartup.value();
                        LevelManagerRequestBus::Broadcast(&LevelManagerRequests::LoadWorld, request);
                    }
                    else
                    {
                        LevelManagerRequestBus::Broadcast(&LevelManagerRequests::UnloadWorld);
                        if (completionCallback)
                        {
                            completionCallback();
                        }
                    }
                });
        };

        // delete spawned entities
        SimulationEntityManagerRequestBus::Broadcast(&SimulationEntityManagerRequests::DeleteAllEntities, deleteAllCompletion);
        return AZ::Success();
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
                if (m_reloadLevelCallback)
                {
                    m_reloadLevelCallback();
                    m_reloadLevelCallback = nullptr;
                }
                if (m_levelLoaded)
                {
                    InitializeSimulationState();
                    m_levelLoaded = false;
                }
                else // transition from other state than load world
                {
                    DeletionCompletedCb deleteAllCompletion = [this](const AZ::Outcome<void, FailedResult>& result)
                    {
                        AZ_Info("SimulationManager", "Delete all entities completed: %s", result.IsSuccess() ? "true" : "false");
                        InitializeSimulationState();
                    };
                    // delete spawned entities
                    SimulationEntityManagerRequestBus::Broadcast(&SimulationEntityManagerRequests::DeleteAllEntities, deleteAllCompletion);
                }

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
        case simulation_interfaces::msg::SimulationState::STATE_NO_WORLD:
            {
                // restore initial state for variables
                m_isSimulationPaused = false;
                m_levelLoaded = false;
                m_numberOfPhysicsSteps = 0;
                break;
            }
        case simulation_interfaces::msg::SimulationState::STATE_LOADING_WORLD:
            {
                m_levelLoaded = true;
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

    bool SimulationManager::EntitiesOperationsPossible()
    {
        return (
            m_simulationState != simulation_interfaces::msg::SimulationState::STATE_LOADING_WORLD &&
            m_simulationState != simulation_interfaces::msg::SimulationState::STATE_NO_WORLD);
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
        // get if we have available keyboard transition

        AZStd::string keyboardHint;
        const auto maybeKeyboardTransition = m_keyboardTransitions.find(m_simulationState);

        if (maybeKeyboardTransition != m_keyboardTransitions.end())
        {
            keyboardHint = maybeKeyboardTransition->second.m_uiDescription;
        }

        DebugDraw::DebugDrawRequestBus::Broadcast(
            &DebugDraw::DebugDrawRequests::DrawTextOnScreen,
            AZStd::string::format("Simulation state: %s \n %s", GetStateName(m_simulationState).c_str(), keyboardHint.c_str()),
            AZ::Color(1.0f, 1.0f, 1.0f, 1.0f),
            0.f);
    }

    void SimulationManager::RegisterTransitionsKey(
        const AzFramework::InputChannelId& key, SimulationState sourceState, SimulationState desiredState)
    {
        const auto uiKeyName = MakePrettyKeyboardName(key);
        const auto uiHint =
            AZStd::string::format("Press %s to change simulation state to %s", uiKeyName.c_str(), GetStateName(desiredState).c_str());
        m_keyboardTransitions[sourceState] = { key, desiredState, uiHint };
    }

    void SimulationManager::UnregisterAllTransitionKeys()
    {
        m_keyboardTransitions.clear();
    }

    bool SimulationManager::OnInputChannelEventFiltered(const AzFramework::InputChannel& inputChannel)
    {
        const AzFramework::InputDeviceId& deviceId = inputChannel.GetInputDevice().GetInputDeviceId();

        if (AzFramework::InputDeviceKeyboard::IsKeyboardDevice(deviceId) && inputChannel.IsStateBegan())
        {
            const auto maybeKeyboardTransition = m_keyboardTransitions.find(m_simulationState);
            if (maybeKeyboardTransition == m_keyboardTransitions.end())
            {
                return false;
            }
            if (maybeKeyboardTransition->second.m_inputChannelId == inputChannel.GetInputChannelId())
            {
                // if we have transition, set the state
                auto result = SetSimulationState(maybeKeyboardTransition->second.m_desiredState);

                AZ_Error(
                    "SimulationManager",
                    result.IsSuccess(),
                    "Failed to change simulation state: %d %s",
                    result.GetError().m_errorCode,
                    result.GetError().m_errorString.c_str());

                return true;
            }
        }
        return false;
    }

} // namespace SimulationInterfaces
