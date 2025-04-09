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
#include <AzFramework/Entity/EntityContextBus.h>
#include <AzFramework/Physics/PhysicsScene.h>
#include <AzFramework/Spawnable/SpawnableEntitiesInterface.h>
#include <SimulationInterfaces/SimulationEntityManagerRequestBus.h>
#include <SimulationInterfaces/SimulationMangerRequestBus.h>
namespace SimulationInterfaces
{
    class SimulationManager
        : public AZ::Component
        , protected SimulationManagerRequestBus::Handler
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
        void Init() override;
        void Activate() override;
        void Deactivate() override;

    protected:
        void SetSimulationPaused(bool paused) override;
        void StepSimulation(AZ::u64 steps) override;
        bool IsSimulationPaused() const override;
        void CancelStepSimulation() override;
        bool IsSimulationStepsActive() const override;

        bool m_isSimulationPaused = false;
        uint64_t m_numberOfPhysicsSteps = 0;
        AzPhysics::SceneEvents::OnSceneSimulationFinishHandler m_simulationFinishEvent;
    };
} // namespace SimulationInterfaces
