/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <SimulationInterfaces/SimulationInterfacesTypeIds.h>

#include <AzCore/EBus/EBus.h>
#include <AzCore/Interface/Interface.h>
#include <AzCore/std/smart_ptr/shared_ptr.h>
#include <AzFramework/Physics/ShapeConfiguration.h>

namespace SimulationInterfaces
{
    //! # A set of filters to apply to entity queries. See GetEntities, GetEntitiesStates.
    //! # The filters are combined with a logical AND.
    //! context : https://github.com/ros-simulation/simulation_interfaces/blob/main/msg/EntityFilters.msg
    struct EntityFilter
    {
        AZStd::string m_filter; //! A posix regular expression to match against entity names
        AZStd::shared_ptr<Physics::ShapeConfiguration>
            m_bounds_shape; //! A shape to use for filtering entities, null means no bounds filtering
        AZ::Transform m_bounds_pose{ AZ::Transform::CreateIdentity() };
    };

    //! context : https://github.com/ros-simulation/simulation_interfaces/blob/main/msg/EntityState.msg
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

    class SimulationInterfacesRequests
    {
    public:
        AZ_RTTI(SimulationInterfacesRequests, SimulationInterfacesRequestsTypeId);
        virtual ~SimulationInterfacesRequests() = default;

        //! # Get a list of entities that match the filter.
        //! Supported filters:
        //! - name : a posix regular expression to match against entity names
        //! - bounds : a shape to use for filtering entities, null means no bounds filtering
        //! context : https://github.com/ros-simulation/simulation_interfaces/blob/main/srv/GetEntities.srv
        virtual AZStd::vector<AZStd::string> GetEntities(const EntityFilter& filter) = 0;

        //! context : https://github.com/ros-simulation/simulation_interfaces/blob/main/srv/GetEntityState.srv
        virtual EntityState GetEntityState(const AZStd::string& name) = 0;

        //! context : https://github.com/ros-simulation/simulation_interfaces/blob/main/srv/GetEntitiesStates.srv
        virtual AZStd::unordered_map<AZStd::string, EntityState> GetEntitiesStates(const EntityFilter& filter) = 0;

        //! context : https://github.com/ros-simulation/simulation_interfaces/blob/main/srv/SetEntityState.srv
        virtual bool SetEntityState(const AZStd::string& name, const EntityState& state) = 0;

        //! context : https://github.com/ros-simulation/simulation_interfaces/blob/main/srv/DeleteEntity.srv
        virtual bool DeleteEntity(const AZStd::string& name) = 0;

        virtual AZStd::vector<Spawnable> GetSpawnables() = 0;

        //! Callback for when an entity has been spawned and registered. The string is the name of the entity in the simulation interface.
        //! Note : The names is empty, if the entity could not be reigstered (e.g. prefab has no simulated entities)
        using SpawnCompletedCb = AZStd::function<void(const AZ::Outcome<AZStd::string, AZStd::string>&)>;

        virtual void SpawnEntity(
            const AZStd::string& name,
            const AZStd::string& uri,
            const AZStd::string& entityNamespace,
            const AZ::Transform& initialPose,
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

    using SimulationInterfacesRequestBus = AZ::EBus<SimulationInterfacesRequests, SimulationInterfacesBusTraits>;
    using SimulationInterfacesInterface = AZ::Interface<SimulationInterfacesRequests>;

} // namespace SimulationInterfaces
