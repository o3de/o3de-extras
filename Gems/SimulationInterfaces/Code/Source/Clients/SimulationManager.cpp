/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "SimulationManager.h"

#include <AzCore/Component/ComponentApplicationBus.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzFramework/Physics/PhysicsSystem.h>
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
    }

    void SimulationManager::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
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
        SimulationManagerRequestBus::Handler::BusConnect();
    }

    void SimulationManager::Deactivate()
    {
        SimulationManagerRequestBus::Handler::BusDisconnect();
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
        }
    }

    void SimulationManager::StepSimulation(AZ::u32 steps)
    {
        m_numberOfPhysicsSteps = steps;

        // install handler
        m_simulationFinishEvent = AzPhysics::SceneEvents::OnSceneSimulationFinishHandler(
            [this](AzPhysics::SceneHandle sceneHandle, float)
            {
                m_numberOfPhysicsSteps--;
                AZ_Printf("SimulationManager", "Physics simulation step finished. Remaining steps: %d", m_numberOfPhysicsSteps);
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

} // namespace SimulationInterfaces
