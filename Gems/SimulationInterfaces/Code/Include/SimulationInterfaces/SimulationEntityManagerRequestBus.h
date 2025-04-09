/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <SimulationInterfaces/SimulationInterfacesTypeIds.h>

#include "Result.h"
#include "TagFilter.h"
#include <AzCore/EBus/EBus.h>
#include <AzCore/Interface/Interface.h>
#include <AzCore/std/smart_ptr/shared_ptr.h>
#include <AzFramework/Physics/ShapeConfiguration.h>

namespace SimulationInterfaces
{
    //! # A set of filters to apply to entity queries. See GetEntities, GetEntitiesStates.
    //! @see <a href="https://github.com/ros-simulation/simulation_interfaces/blob/main/msg/EntityFilters.msg">EntityFilters.msg</a>
    struct EntityFilters
    {
        AZStd::string m_filter; //! A posix regular expression to match against entity names
        TagFilter m_tags_filter; //! A filter to match against entity tags
        AZStd::shared_ptr<Physics::ShapeConfiguration>
            m_bounds_shape; //! A shape to use for filtering entities, null means no bounds filtering
        AZ::Transform m_bounds_pose{ AZ::Transform::CreateIdentity() };
    };

    //!  @see <a href="https://github.com/ros-simulation/simulation_interfaces/blob/main/msg/EntityState.msg">EntityState.msg</a>
    struct EntityState
    {
        AZ::Transform m_pose; //! The pose of the entity
        AZ::Vector3 m_twist_linear; //! The linear velocity of the entity (in the entity frame)
        AZ::Vector3 m_twist_angular; //! The angular velocity of the entity (in the entity frame)
    };

    struct Spawnable
    {
        AZStd::string m_uri;
        AZStd::string m_description;
        AZStd::string m_bounds_sphere;
    };

    using EntityNameList = AZStd::vector<AZStd::string>;
    using MultipleEntitiesStates = AZStd::unordered_map<AZStd::string, EntityState>;
    using SpawnableList = AZStd::vector<Spawnable>;
    using DeletionCompletedCb = AZStd::function<void(const AZ::Outcome<void, FailedResult>&)>;

    struct SpawnedEntities {
        AZStd::string m_name; //! The resulting name of spawnable
        AZStd::vector<AZ::EntityId> m_entityIds; //! The list of entity ids that were spawned
    };

    using SpawnCompletedCb = AZStd::function<void(const AZ::Outcome<SpawnedEntities, FailedResult>&)>;
    class SimulationEntityManagerRequests
    {
    public:
        AZ_RTTI(SimulationEntityManagerRequests, SimulationInterfacesRequestsTypeId);
        virtual ~SimulationEntityManagerRequests() = default;

        //! # Get a list of entities that match the filter.
        //! Supported filters:
        //! - name : a posix regular expression to match against entity names
        //! - bounds : a shape to use for filtering entities, null means no bounds filtering
        //!  @see <a href="https://github.com/ros-simulation/simulation_interfaces/blob/main/srv/GetEntities.srv">GetEntities.srv</a>
        virtual AZ::Outcome<EntityNameList, FailedResult> GetEntities(const EntityFilters& filter) = 0;

        //! Get the state of an entity.
        //!  @see <a href="https://github.com/ros-simulation/simulation_interfaces/blob/main/srv/GetEntityState.srv">GetEntityState.srv</a>
        virtual AZ::Outcome<EntityState, FailedResult> GetEntityState(const AZStd::string& name) = 0;

        //! Get the state of all entities that match the filter.
        //! @see <a
        //! href="https://github.com/ros-simulation/simulation_interfaces/blob/main/srv/GetEntitiesStates.srv">GetEntitiesStates.srv</a>
        virtual AZ::Outcome<MultipleEntitiesStates, FailedResult> GetEntitiesStates(const EntityFilters& filter) = 0;

        //! Set the state of an entity.
        //! @see <a href="https://github.com/ros-simulation/simulation_interfaces/blob/main/srv/SetEntityState.srv">SetEntityState.srv</a>
        virtual AZ::Outcome<void, FailedResult> SetEntityState(const AZStd::string& name, const EntityState& state) = 0;

        //! Remove previously spawned entity from the simulation.
        //! @see <a href="https://github.com/ros-simulation/simulation_interfaces/blob/main/srv/DeleteEntity.srv">DeleteEntity.srv</a>
        virtual void DeleteEntity(const AZStd::string& name, DeletionCompletedCb completedCb) = 0;

        //! Get a list of spawnable entities.
        //! @see <a href="https://github.com/ros-simulation/simulation_interfaces/blob/main/srv/GetSpawnables.srv">GetSpawnables.srv</a>
        virtual AZ::Outcome<SpawnableList, FailedResult> GetSpawnables() = 0;

        //! Callback for when an entity has been spawned and registered. The string is the name of the entity in the simulation interface.
        //! Note : The names is empty, if the entity could not be registered (e.g. prefab has no simulated entities)
        virtual void SpawnEntity(
            const AZStd::string& name,
            const AZStd::string& uri,
            const AZ::Transform& initialPose,
            const bool allowRename,
            SpawnCompletedCb completedCb) = 0;
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
