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

namespace SimulationInterfaces
{
    class SimulationEntitiesManager
        : public AZ::Component
        , protected SimulationEntityManagerRequestBus::Handler
    {
    public:
        AZ_COMPONENT_DECL(SimulationEntitiesManager);

        static void Reflect(AZ::ReflectContext* context);

        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);

        SimulationEntitiesManager();
        ~SimulationEntitiesManager();

    protected:
        // SimulationEntityManagerRequestBus interface implementation
        AZ::Outcome<EntityNameList, FailedResult> GetEntities(const EntityFilters& filter) override;
        AZ::Outcome<EntityState, FailedResult> GetEntityState(const AZStd::string& name) override;
        AZ::Outcome<MultipleEntitiesStates, FailedResult> GetEntitiesStates(const EntityFilters& filter) override;
        AZ::Outcome<void, FailedResult> SetEntityState(const AZStd::string& name, const EntityState& state) override;
        void DeleteEntity(const AZStd::string& name, DeletionCompletedCb completedCb) override;
        void DeleteAllEntities(DeletionCompletedCb completedCb) override;
        AZ::Outcome<SpawnableList, FailedResult> GetSpawnables() override;
        void SpawnEntity(
            const AZStd::string& name,
            const AZStd::string& uri,
            const AZStd::string& entityNamespace,
            const AZ::Transform& initialPose,
            const bool allowRename,
            SpawnCompletedCb completedCb) override;
        void ResetAllEntitiesToInitialState() override;

        // AZ::Component interface implementation
        void Init() override;
        void Activate() override;
        void Deactivate() override;

    private:

        //! Registers simulated entity to entity id mapping.
        //! Note that the entityId will be registered under unique name.
        //! \param entityId The entity id to register
        //! \param proposedName Optional user proposed name for the simulated entity
        //! \return returns the simulated entity name
        AZStd::string AddSimulatedEntity(AZ::EntityId entityId, const AZStd::string& proposedName);

        //! Removes simulated entity from the mapping.
        void RemoveSimulatedEntity(AZ::EntityId entityId);

        //! Returns the simulated entity name for the given entity id.
        AZStd::string GetSimulatedEntityName(AZ::EntityId entityId, const AZStd::string& proposedName) const;

        //! Set the state of the entity and their descendants.
        void SetEntitiesState(const AZStd::vector<AZ::EntityId>& entityAndDescendants, const EntityState& state);

        bool m_registerEverySimulatedEntity = false; //! Flag to register every simulated entity

        AzPhysics::SceneEvents::OnSimulationBodyAdded::Handler m_simulationBodyAddedHandler;
        AzPhysics::SceneEvents::OnSimulationBodyRemoved::Handler m_simulationBodyRemovedHandler;

        AzPhysics::SystemEvents::OnSceneAddedEvent::Handler m_sceneAddedHandler;
        AzPhysics::SystemEvents::OnSceneRemovedEvent::Handler m_sceneRemovedHandler;
        AzPhysics::SceneHandle m_physicsScenesHandle = AzPhysics::InvalidSceneHandle;
        AZStd::unordered_map<AZStd::string, AZ::EntityId> m_simulatedEntityToEntityIdMap;
        AZStd::unordered_map<AZ::EntityId, AZStd::string> m_entityIdToSimulatedEntityMap;
        AZStd::unordered_map<AZ::EntityId, EntityState> m_entityIdToInitialState;
        AZStd::unordered_map<AzFramework::EntitySpawnTicket::Id, AzFramework::EntitySpawnTicket> m_spawnedTickets;

        struct SpawnCompletedCbData
        {
            AZStd::string m_userProposedName; //! Name proposed by the User in spawn request
            AZStd::string m_resultedName; //! Name of the entity in the simulation interface
            SpawnCompletedCb m_completedCb; //! User callback to be called when the entity is registered
            AZ::ScriptTimePoint m_spawnCompletedTime; //! Time at which the entity was spawned
            bool m_registered = false; //! Flag to check if the entity was registered
        };
        AZStd::unordered_map<AzFramework::EntitySpawnTicket::Id, SpawnCompletedCbData>
            m_spawnCompletedCallbacks; //! Callbacks to be called when the entity is registered
    };

} // namespace SimulationInterfaces
