/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "SimulationManager.h"
#include "SimulationInterfaces/SimulationFeatures.h"

#include <AzCore/Component/ComponentApplicationBus.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzFramework/Components/ConsoleBus.h>
#include <AzFramework/Physics/PhysicsSystem.h>
#include <SimulationInterfaces/SimulationFeaturesAggregatorRequestBus.h>
#include <SimulationInterfaces/SimulationInterfacesTypeIds.h>

namespace SimulationInterfaces
{

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

    void SimulationManager::Init()
    {
    }

    void SimulationManager::Activate()
    {
        AzFramework::LevelSystemLifecycleNotificationBus::Handler::BusDisconnect();
        SimulationManagerRequestBus::Handler::BusConnect();
        SimulationFeaturesAggregatorRequestBus::Broadcast(
            &SimulationFeaturesAggregatorRequests::AddSimulationFeatures,
            AZStd::unordered_set<SimulationFeatures>{
                SimulationFeatures::SIMULATION_RESET,
                SimulationFeatures::SIMULATION_RESET_TIME,
                //SimulationFeatures::SIMULATION_RESET_STATE,
                SimulationFeatures::SIMULATION_RESET_SPAWNED,
                SimulationFeatures::SIMULATION_STATE_PAUSE,
                SimulationFeatures::STEP_SIMULATION_SINGLE,
                SimulationFeatures::STEP_SIMULATION_MULTIPLE,
                SimulationFeatures::STEP_SIMULATION_ACTION});
    }

    void SimulationManager::Deactivate()
    {
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
        auto* sceneInterface = AZ::Interface<AzPhysics::SceneInterface>::Get();
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
                SimulationManagerNotificationsBus::Broadcast(&SimulationManagerNotifications::OnSimulationStepFinish, m_numberOfPhysicsSteps);
                if (m_numberOfPhysicsSteps <= 0)
                {
                    SetSimulationPaused(true);
                    // remove handler
                    m_simulationFinishEvent.Disconnect();
                }
            });

        // get default scene
        auto* physicsSystem = AZ::Interface<AzPhysics::SystemInterface>::Get();
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
        AzFramework::LevelSystemLifecycleNotificationBus::Handler::BusDisconnect();
    }

} // namespace SimulationInterfaces
