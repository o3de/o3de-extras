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
#include "TagFilter.h"
#include <AzCore/EBus/EBus.h>
#include <AzCore/Interface/Interface.h>
#include <AzCore/std/smart_ptr/shared_ptr.h>
#include <AzFramework/Physics/ShapeConfiguration.h>

namespace SimulationInterfaces
{
    //! A set of filters to apply to entity queries. See GetEntities, GetEntitiesStates.
    //! @see <a href="https://github.com/ros-simulation/simulation_interfaces/blob/main/msg/EntityFilters.msg">EntityFilters.msg</a>
    struct EntityFilters
    {
        //! A posix regular expression to match against entity names,
        //! The regular expression syntax is POSIX Extended,
        //! @see <a href="https://pubs.opengroup.org/onlinepubs/9799919799">POSIX_Extended</a> definitions
        AZStd::string m_nameFilter;
        TagFilter m_tagsFilter; //! A filter to match against entity tags
        AZStd::shared_ptr<Physics::ShapeConfiguration>
            m_boundsShape; //! A shape to use for filtering entities, null means no bounds filtering
        AZ::Transform m_boundsPose{ AZ::Transform::CreateIdentity() };
    };

    //! @see <a href="https://github.com/ros-simulation/simulation_interfaces/blob/main/msg/EntityState.msg">EntityState.msg</a>
    struct EntityState
    {
        AZ::Transform m_pose; //! The pose of the entity
        AZ::Vector3 m_twistLinear; //! The linear velocity of the entity (in the entity frame)
        AZ::Vector3 m_twistAngular; //! The angular velocity of the entity (in the entity frame)
    };

    //! @see <a href="https://github.com/ros-simulation/simulation_interfaces/blob/main/msg/Spawnable.msg">Spawnable.msg</a>
    struct Spawnable
    {
        AZStd::string m_uri;
        AZStd::string m_description;
        AZStd::string m_boundsSphere;
    };

    using EntityNameList = AZStd::vector<AZStd::string>;
    using MultipleEntitiesStates = AZStd::unordered_map<AZStd::string, EntityState>;
    using SpawnableList = AZStd::vector<Spawnable>;
    using DeletionCompletedCb = AZStd::function<void(const AZ::Outcome<void, FailedResult>&)>;
    using SpawnCompletedCb = AZStd::function<void(const AZ::Outcome<AZStd::string, FailedResult>&)>;

    class SimulationEntityManagerRequests
    {
    public:
        AZ_RTTI(SimulationEntityManagerRequests, SimulationInterfacesRequestsTypeId);
        virtual ~SimulationEntityManagerRequests() = default;

        //! # Get a list of entities that match the filter.
        //! Supported filters:
        //! - name : a posix regular expression to match against entity names
        //! - bounds : a shape to use for filtering entities, null means no bounds filtering
        //! @see <a href="https://github.com/ros-simulation/simulation_interfaces/blob/main/srv/GetEntities.srv">GetEntities.srv</a>
        virtual AZ::Outcome<EntityNameList, FailedResult> GetEntities(const EntityFilters& filter) = 0;

        //! Get the state of an entity.
        //! @see <a href="https://github.com/ros-simulation/simulation_interfaces/blob/main/srv/GetEntityState.srv">GetEntityState.srv</a>
        virtual AZ::Outcome<EntityState, FailedResult> GetEntityState(const AZStd::string& name) = 0;

        //! Get the state of all entities that match the filter.
        //! @see <a href="https://github.com/ros-simulation/simulation_interfaces/blob/main/srv/GetEntitiesStates.srv">GetEntitiesStates.srv</a>
        virtual AZ::Outcome<MultipleEntitiesStates, FailedResult> GetEntitiesStates(const EntityFilters& filter) = 0;

        //! Set the state of an entity.
        //! @see <a href="https://github.com/ros-simulation/simulation_interfaces/blob/main/srv/SetEntityState.srv">SetEntityState.srv</a>
        virtual AZ::Outcome<void, FailedResult> SetEntityState(const AZStd::string& name, const EntityState& state) = 0;

        //! Remove previously spawned entity from the simulation.
        //! @see <a href="https://github.com/ros-simulation/simulation_interfaces/blob/main/srv/DeleteEntity.srv">DeleteEntity.srv</a>
        virtual void DeleteEntity(const AZStd::string& name, DeletionCompletedCb completedCb) = 0;

        //! Remove all previously spawned entity from the simulation.
        virtual void DeleteAllEntities(DeletionCompletedCb completedCb) = 0;

        //! Get a list of spawnable entities.
        //! @see <a href="https://github.com/ros-simulation/simulation_interfaces/blob/main/srv/GetSpawnables.srv">GetSpawnables.srv</a>
        virtual AZ::Outcome<SpawnableList, FailedResult> GetSpawnables() = 0;

        //! Callback for when an entity has been spawned and registered. The string is the name of the entity in the simulation interface.
        //! Note: The names are empty, if the entity could not be registered (e.g. prefab has no simulated entities)
        virtual void SpawnEntity(
            const AZStd::string& name,
            const AZStd::string& uri,
            const AZStd::string& entityNamespace,
            const AZ::Transform& initialPose,
            const bool allowRename,
            SpawnCompletedCb completedCb) = 0;

        //! Reset the simulation to begin.
        //! This will revert the entire simulation to the initial state.
        virtual void ResetAllEntitiesToInitialState() = 0;
    };

    class SimulationInterfacesBusTraits : public AZ::EBusTraits
    {
    public:
        //////////////////////////////////////////////////////////////////////////
        // EBusTraits overrides
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::Single;
        //////////////////////////////////////////////////////////////////////////
    };

    using SimulationEntityManagerRequestBus = AZ::EBus<SimulationEntityManagerRequests, SimulationInterfacesBusTraits>;
    using SimulationEntityManagerInterface = AZ::Interface<SimulationEntityManagerRequests>;

} // namespace SimulationInterfaces
