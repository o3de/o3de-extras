/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "SimulationEntitiesManager.h"
#include "CommonUtilities.h"
#include <AzCore/Asset/AssetManager.h>
#include <AzCore/Asset/AssetManagerBus.h>
#include <AzCore/Component/ComponentApplicationBus.h>
#include <AzCore/Component/EntityId.h>
#include <AzCore/Component/TransformBus.h>
#include <AzCore/Console/IConsole.h>
#include <AzCore/Math/Obb.h>
#include <AzCore/Outcome/Outcome.h>
#include <AzCore/RTTI/RTTIMacros.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/Settings/SettingsRegistry.h>
#include <AzCore/std/algorithm.h>
#include <AzCore/std/containers/vector.h>
#include <AzCore/std/ranges/elements_view.h>
#include <AzCore/std/ranges/ranges_algorithm.h>
#include <AzCore/std/string/regex.h>
#include <AzCore/std/string/string.h>
#include <AzFramework/Components/TransformComponent.h>
#include <AzFramework/Physics/Common/PhysicsSceneQueries.h>
#include <AzFramework/Physics/Common/PhysicsSimulatedBody.h>
#include <AzFramework/Physics/PhysicsSystem.h>
#include <AzFramework/Physics/RigidBodyBus.h>
#include <AzFramework/Physics/SimulatedBodies/RigidBody.h>
#include <AzFramework/Spawnable/Spawnable.h>
#include <AzFramework/Spawnable/SpawnableEntitiesInterface.h>
#include <AzFramework/Entity/EntityContextBus.h>
#include <AzFramework/Entity/GameEntityContextBus.h>
#include <ROS2/Frame/ROS2FrameComponent.h>
#include <SimulationInterfaces/Bounds.h>
#include <SimulationInterfaces/Result.h>
#include <SimulationInterfaces/SimulationFeaturesAggregatorRequestBus.h>
#include <SimulationInterfaces/SimulationInterfacesTypeIds.h>
#include <SimulationInterfaces/SimulationMangerRequestBus.h>
#include <simulation_interfaces/msg/result.hpp>
#include <simulation_interfaces/msg/simulator_features.hpp>
#include <simulation_interfaces/srv/spawn_entity.hpp>

namespace SimulationInterfaces
{
    namespace
    {
        // copied from PhysX Gem to prevent adding dependency to PhysX (Gems/PhysX/Core/Code/Include/PhysX/NativeTypeIdentifiers.h)
        static const AZ::Crc32 RigidBody = AZ_CRC_CE("PhysXRigidBody");
        static const AZ::Crc32 RigidBodyStatic = AZ_CRC_CE("PhysXRigidBodyStatic");
        static const AZ::Crc32 ArticulationLink = AZ_CRC_CE("PhysXArticulationLink");

        void SetRigidBodyVelocities(AzPhysics::RigidBody* rigidBody, const EntityState& state)
        {
            if (!state.m_twistAngular.IsClose(AZ::Vector3::CreateZero(), AZ::Constants::FloatEpsilon))
            {
                // get transform
                AZ::Vector3 angularVelWorld = rigidBody->GetTransform().TransformVector(state.m_twistAngular);
                rigidBody->SetAngularVelocity(angularVelWorld);
            }

            if (!state.m_twistLinear.IsClose(AZ::Vector3::CreateZero(), AZ::Constants::FloatEpsilon))
            {
                // get transform
                AZ::Vector3 linearVelWorld = rigidBody->GetTransform().TransformVector(state.m_twistLinear);
                rigidBody->SetLinearVelocity(linearVelWorld);
            }
        }

    } // namespace

    AZ_COMPONENT_IMPL(SimulationEntitiesManager, "SimulationEntitiesManager", SimulationEntitiesManagerTypeId);

    void SimulationEntitiesManager::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<SimulationEntitiesManager, AZ::Component>()->Version(0);
        }
    }

    void SimulationEntitiesManager::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("SimulationInterfacesService"));
    }

    void SimulationEntitiesManager::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("SimulationInterfacesService"));
    }

    void SimulationEntitiesManager::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("AssetCatalogService"));
        required.push_back(AZ_CRC_CE("SimulationManagerService"));
        required.push_back(AZ_CRC_CE("SimulationFeaturesAggregator"));
    }

    void SimulationEntitiesManager::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
        dependent.push_back(AZ_CRC_CE("PhysicsService"));
        dependent.push_back(AZ_CRC_CE("SimulationManagerService"));
        dependent.push_back(AZ_CRC_CE("SimulationFeaturesAggregator"));
    }

    SimulationEntitiesManager::SimulationEntitiesManager()
    {
        if (SimulationEntityManagerInterface::Get() == nullptr)
        {
            SimulationEntityManagerInterface::Register(this);
        }
    }

    SimulationEntitiesManager::~SimulationEntitiesManager()
    {
        if (SimulationEntityManagerInterface::Get() == this)
        {
            SimulationEntityManagerInterface::Unregister(this);
        }
    }

    AZ::Outcome<AZStd::string, FailedResult> SimulationEntitiesManager::RegisterNewSimulatedBody(
        const AZStd::string& proposedName, const AZ::EntityId& entityId)
    {
        if (!entityId.IsValid())
        {
            constexpr const char* msg = "EntityId is not valid";
            AZ_Trace("SimulationInterfaces", msg);
            return AZ::Failure(FailedResult(simulation_interfaces::msg::Result::RESULT_OPERATION_FAILED, msg));
        }
        AZ::Entity* entity = nullptr;
        AZ::ComponentApplicationBus::BroadcastResult(entity, &AZ::ComponentApplicationRequests::FindEntity, entityId);
        if (entity == nullptr)
        {
            constexpr const char* msg = "Entity pointer is not valid";
            AZ_Trace("SimulationInterfaces", msg);
            return AZ::Failure(FailedResult(simulation_interfaces::msg::Result::RESULT_OPERATION_FAILED, msg));
        }

        // register entity
        const AZStd::string registeredName = this->AddSimulatedEntity(entityId, proposedName);

        // cache the initial state - for simulator reset with SCOPE_STATE.
        EntityState initialState{};
        initialState.m_pose = entity->GetTransform()->GetWorldTM();
        auto simulatedBody = Utils::GetSimulatedBody(entityId);
        if (simulatedBody.IsSuccess())
        {
            if (auto* rigidBody = azdynamic_cast<AzPhysics::RigidBody*>(simulatedBody.GetValue()))
            {
                initialState.m_twistLinear = rigidBody->GetLinearVelocity();
                initialState.m_twistAngular = rigidBody->GetAngularVelocity();
            }
        }

        m_entityIdToInitialState[entityId] = initialState;
        AZ_Info("SimulationInterfaces", "Registered entity %s\n", registeredName.c_str());
        return AZ::Success(registeredName);
    }

    void SimulationEntitiesManager::Activate()
    {
        m_sceneAddedHandler = AzPhysics::SystemEvents::OnSceneAddedEvent::Handler(
            [this](AzPhysics::SceneHandle sceneHandle)
            {
                AZ_Warning(
                    "SimulationInterfaces",
                    m_physicsScenesHandle == AzPhysics::InvalidSceneHandle,
                    "SimulationInterfaces already gathered physics scene");
                m_physicsScenesHandle = sceneHandle;
            });
        m_sceneRemovedHandler = AzPhysics::SystemEvents::OnSceneRemovedEvent::Handler(
            [this](AzPhysics::SceneHandle sceneHandle)
            {
                if (m_physicsScenesHandle == sceneHandle)
                {
                    AZ_Info("SimulationInterfaces", "Physics scene handler removed");
                    m_physicsScenesHandle = AzPhysics::InvalidSceneHandle;
                }
            });
        AzPhysics::SystemInterface* physicsSystem = AZ::Interface<AzPhysics::SystemInterface>::Get();
        if (physicsSystem)
        {
            physicsSystem->RegisterSceneAddedEvent(m_sceneAddedHandler);
            physicsSystem->RegisterSceneRemovedEvent(m_sceneRemovedHandler);
        }

        SimulationEntityManagerRequestBus::Handler::BusConnect();
        SimulationFeaturesAggregatorRequestBus::Broadcast(
            &SimulationFeaturesAggregatorRequests::AddSimulationFeatures,
            AZStd::unordered_set<SimulationFeatureType>{
                simulation_interfaces::msg::SimulatorFeatures::ENTITY_TAGS,
                simulation_interfaces::msg::SimulatorFeatures::ENTITY_BOUNDS_BOX,
                // not implemented: simulation_interfaces::msg::SimulatorFeatures::ENTITY_BOUNDS_CONVEX,
                simulation_interfaces::msg::SimulatorFeatures::ENTITY_CATEGORIES,
                simulation_interfaces::msg::SimulatorFeatures::ENTITY_STATE_GETTING,
                simulation_interfaces::msg::SimulatorFeatures::ENTITY_STATE_SETTING,
                simulation_interfaces::msg::SimulatorFeatures::ENTITY_INFO_GETTING,
                simulation_interfaces::msg::SimulatorFeatures::ENTITY_INFO_SETTING,
                simulation_interfaces::msg::SimulatorFeatures::ENTITY_BOUNDS,
                simulation_interfaces::msg::SimulatorFeatures::DELETING,
                simulation_interfaces::msg::SimulatorFeatures::SPAWNABLES,
                simulation_interfaces::msg::SimulatorFeatures::SPAWNING });
    }

    void SimulationEntitiesManager::Deactivate()
    {
        SimulationEntityManagerRequestBus::Handler::BusDisconnect();

        m_physicsScenesHandle = AzPhysics::InvalidSceneHandle;
        m_sceneAddedHandler.Disconnect();
        m_sceneRemovedHandler.Disconnect();

        m_entityIdToSimulatedEntityMap.clear();
        m_simulatedEntityToEntityIdMap.clear();
        m_simulatedEntityToPrefabRoot.clear();
        m_entityIdToInitialState.clear();
        m_spawnedTickets.clear();
    }

    AZStd::string SimulationEntitiesManager::AddSimulatedEntity(AZ::EntityId entityId, const AZStd::string& userProposedName)
    {
        if (!entityId.IsValid())
        {
            return "";
        }
        // check if entity is already registered
        auto findIt = m_entityIdToSimulatedEntityMap.find(entityId);
        if (findIt != m_entityIdToSimulatedEntityMap.end())
        {
            return findIt->second;
        }
        // register entity under unique name
        AZStd::string simulatedEntityName = GetSimulatedEntityName(entityId, userProposedName);
        m_simulatedEntityToEntityIdMap[simulatedEntityName] = entityId;
        m_entityIdToSimulatedEntityMap[entityId] = simulatedEntityName;
        return simulatedEntityName;
    }

    AZ::Outcome<void, FailedResult> SimulationEntitiesManager::UnregisterSimulatedBody(const AZStd::string& name)
    {
        if (auto findIt = m_simulatedEntityToEntityIdMap.find(name); findIt != m_simulatedEntityToEntityIdMap.end())
        {
            // remove registry
            const auto& entityId = findIt->second;
            // remove initial state
            if (auto findStateIt = m_entityIdToInitialState.find(entityId); findStateIt != m_entityIdToInitialState.end())
            {
                m_entityIdToInitialState.erase(findStateIt);
            }
            m_entityIdToSimulatedEntityMap.erase(entityId);
            m_simulatedEntityToEntityIdMap.erase(findIt);

            RemoveEntityInfoIfNeeded(name);

            // remove info about root of simulated entity only if it was spawned by this component
            if (m_simulatedEntityToPrefabRoot.contains(name))
            {
                m_simulatedEntityToPrefabRoot.erase(name);
            }

            return AZ::Success();
        }
        return AZ::Failure(FailedResult(
            simulation_interfaces::msg::Result::RESULT_OPERATION_FAILED,
            AZStd::string::format("Failed to find entity with given name %s", name.c_str())));
    }

    AZStd::vector<AZStd::string> SimulationEntitiesManager::FilterEntitiesByCategories(
        const AZStd::vector<AZStd::string>& prefilteredEntities, const AZStd::vector<EntityCategory>& categories)
    {
        AZStd::vector<AZStd::string> entities;
        entities.reserve(prefilteredEntities.size());
        for (auto& category : categories)
        {
            const auto categoryExists = m_categoryToNames.contains(category);
            AZ_Warning(
                "SpawnableSceneProviderSystemComponent",
                categoryExists,
                "Category %d doesn't exists in database, will be skipped",
                category);
            if (categoryExists)
            {
                AZStd::ranges::copy_if(
                    prefilteredEntities,
                    AZStd::back_inserter(entities),
                    [this, category](const AZStd::string& entityName)
                    {
                        return m_categoryToNames.at(category).contains(entityName);
                    });
            }
        }
        return entities;
    }

    AZStd::vector<AZStd::string> SimulationEntitiesManager::FilterEntitiesByTag(
        const AZStd::vector<AZStd::string>& prefilteredEntities, const TagFilter& tagFilter)
    {
        AZStd::vector<AZStd::string> entities;
        entities.reserve(prefilteredEntities.size());
        for (const auto& name : prefilteredEntities)
        {
            // if entity doesn't have entity info it cannot be filtered by tag so it should be skipped
            auto findIt = m_nameToEntityInfo.find(name);
            if (findIt == m_nameToEntityInfo.end())
            {
                continue;
            }
            // get entity tags
            if (Utils::AreTagsMatching(tagFilter, findIt->second.m_tags))
            {
                entities.push_back(name);
            }
        }
        return entities;
    }

    AZ::Outcome<AZStd::vector<AZStd::string>, FailedResult> SimulationEntitiesManager::FilterEntitiesByBounds(
        const AZStd::vector<AZStd::string>& prefilteredEntities,
        const AZStd::shared_ptr<Physics::ShapeConfiguration> shape,
        const AZ::Transform& shapePose)
    {
        auto* sceneInterface = AZ::Interface<AzPhysics::SceneInterface>::Get();
        AZ_Assert(sceneInterface, "Physics scene interface is not available.");

        if (m_physicsScenesHandle == AzPhysics::InvalidSceneHandle)
        {
            return AZ::Failure(
                FailedResult(simulation_interfaces::msg::Result::RESULT_OPERATION_FAILED, "Physics scene interface is not available."));
        }

        AzPhysics::OverlapRequest request;
        request.m_shapeConfiguration = shape;
        request.m_pose = shapePose;
        request.m_maxResults = AZStd::numeric_limits<AZ::u32>::max();
        AzPhysics::SceneQueryHits result = sceneInterface->QueryScene(m_physicsScenesHandle, &request);
        AZStd::vector<AZStd::string> entities;
        entities.reserve(prefilteredEntities.size());

        for (const auto& name : prefilteredEntities)
        {
            const auto& entityId = m_simulatedEntityToEntityIdMap.at(name);
            AZ::Outcome<AzPhysics::SimulatedBody*, AZStd::string> simulatedBody = Utils::GetSimulatedBody(entityId);
            if (simulatedBody.IsSuccess())
            {
                auto rigidBody = azdynamic_cast<AzPhysics::RigidBody*>(simulatedBody.GetValue());
                // entity is simulated body and has collider, check overlap
                if (rigidBody && rigidBody->GetShapeCount() > 0)
                {
                    // perform relatively expensive search only for entities which really can be inside the shape
                    if (AZStd::ranges::contains(result.m_hits, entityId, &AzPhysics::SceneQueryHit::m_entityId))
                    {
                        entities.push_back(name);
                    }
                    // no matter of the results skip the rest of the loop since it is related to worldTM check
                    continue;
                }
            }

            // for non physical or no-colliding entities check if World TM is inside the control shape
            AZ::Vector3 worldTranslation;
            AZ::TransformBus::EventResult(worldTranslation, entityId, &AZ::TransformBus::Events::GetWorldTranslation);
            // check if within bounds
            if (auto boxShape = dynamic_cast<Physics::BoxShapeConfiguration*>(shape.get()))
            {
                auto oob = boxShape->ToObb(shapePose);
                if (oob.Contains(worldTranslation))
                {
                    entities.push_back(name);
                }
            }
            else if (auto sphereShape = dynamic_cast<Physics::SphereShapeConfiguration*>(shape.get()))
            {
                auto distance = shapePose.GetTranslation().GetDistance(worldTranslation);
                if (distance <= sphereShape->m_radius)
                {
                    entities.push_back(name);
                }
            }
            else
            {
                AZ_Warning("SimulationInterfaces", false, "Unsupported bounds type, skipped");
            }
        }
        return entities;
    }

    AZ::Outcome<AZStd::vector<AZStd::string>, FailedResult> SimulationEntitiesManager::FilterEntitiesByRegex(
        const AZStd::vector<AZStd::string>& prefilteredEntities, const AZStd::string& nameRegex)
    {
        AZStd::vector<AZStd::string> entities;
        entities.reserve(prefilteredEntities.size());
        const AZStd::regex regexSearch(nameRegex, AZStd::regex::extended);
        if (!regexSearch.Valid())
        {
            AZ_Warning("SimulationInterfaces", false, "Invalid regex filter");
            return AZ::Failure(FailedResult(simulation_interfaces::msg::Result::RESULT_NOT_FOUND, "Invalid regex filter"));
        }
        AZStd::ranges::copy_if(
            prefilteredEntities,
            AZStd::back_inserter(entities),
            [&regexSearch](const AZStd::string& entityName)
            {
                return AZStd::regex_search(entityName, regexSearch);
            });
        return entities;
    }

    AZ::Outcome<EntityNameList, FailedResult> SimulationEntitiesManager::GetEntities(const EntityFilters& filter)
    {
        if (auto outcome = IsWorldLoaded(); !outcome.IsSuccess())
        {
            return AZ::Failure(outcome.GetError());
        }

        const bool reFilter = !filter.m_nameFilter.empty();
        const bool shapeCastFilter = filter.m_boundsShape != nullptr;
        const bool categoriesFilter = !filter.m_entityCategories.empty();
        const bool tagFilter = !filter.m_tagsFilter.m_tags.empty();

        // get all entities from the map
        const auto valueView = AZStd::ranges::views::values(m_entityIdToSimulatedEntityMap);
        AZStd::vector<AZStd::string> entities{ valueView.begin(), valueView.end() };

        // filter by categories
        if (categoriesFilter)
        {
            entities = FilterEntitiesByCategories(entities, filter.m_entityCategories);
        }

        // filter based on tag
        if (tagFilter)
        {
            entities = FilterEntitiesByTag(entities, filter.m_tagsFilter);
        }

        if (shapeCastFilter)
        {
            auto outcome = FilterEntitiesByBounds(entities, filter.m_boundsShape, filter.m_boundsPose);
            if (!outcome.IsSuccess())
            {
                return AZ::Failure(outcome.GetError());
            }
            entities = outcome.GetValue();
        }

        if (reFilter)
        {
            auto outcome = FilterEntitiesByRegex(entities, filter.m_nameFilter);

            if (!outcome.IsSuccess())
            {
                return AZ::Failure(outcome.GetError());
            }

            entities = outcome.GetValue();
        }
        return AZ::Success(entities);
    }

    AZ::Outcome<EntityState, FailedResult> SimulationEntitiesManager::GetEntityState(const AZStd::string& name)
    {
        if (auto outcome = IsWorldLoaded(); !outcome.IsSuccess())
        {
            return AZ::Failure(outcome.GetError());
        }

        const auto findIt = m_simulatedEntityToEntityIdMap.find(name);
        if (findIt == m_simulatedEntityToEntityIdMap.end())
        {
            AZ_Warning("SimulationInterfaces", false, "Entity %s not found", name.c_str());
            return AZ::Failure(FailedResult(simulation_interfaces::msg::Result::RESULT_NOT_FOUND, "Entity not found"));
        }
        EntityState entityState{};
        const AZ::EntityId entityId = findIt->second;
        AZ_Assert(entityId.IsValid(), "EntityId is not valid");

        AZ::TransformBus::EventResult(entityState.m_pose, entityId, &AZ::TransformBus::Events::GetWorldTM);

        AZ::Vector3 linearVelocity = AZ::Vector3::CreateZero();
        Physics::RigidBodyRequestBus::EventResult(linearVelocity, entityId, &Physics::RigidBodyRequests::GetLinearVelocity);

        AZ::Vector3 angularVelocity = AZ::Vector3::CreateZero();
        Physics::RigidBodyRequestBus::EventResult(angularVelocity, entityId, &Physics::RigidBodyRequests::GetAngularVelocity);

        // transform linear and angular velocities to entity frame
        const AZ::Transform entityTransformInv = entityState.m_pose.GetInverse();
        entityState.m_twistLinear = entityTransformInv.TransformVector(linearVelocity);
        entityState.m_twistAngular = entityTransformInv.TransformVector(angularVelocity);
        return AZ::Success(entityState);
    }

    AZ::Outcome<MultipleEntitiesStates, FailedResult> SimulationEntitiesManager::GetEntitiesStates(const EntityFilters& filter)
    {
        if (auto outcome = IsWorldLoaded(); !outcome.IsSuccess())
        {
            return AZ::Failure(outcome.GetError());
        }

        if (!filter.m_tagsFilter.m_tags.empty())
        {
            AZ_Warning("SimulationInterfaces", false, "Tags filter is not implemented yet");
            return AZ::Failure(
                FailedResult(simulation_interfaces::msg::Result::RESULT_FEATURE_UNSUPPORTED, "Tags filter is not implemented yet"));
        }
        MultipleEntitiesStates entitiesStates;
        const auto& entities = GetEntities(filter);
        if (!entities.IsSuccess())
        {
            return AZ::Failure(entities.GetError());
        }
        for (const auto& entity : entities.GetValue())
        {
            auto state = GetEntityState(entity);
            if (!state.IsSuccess())
            {
                return AZ::Failure(state.GetError());
            }
            entitiesStates.emplace(AZStd::make_pair(entity, state.GetValue()));
        }
        return entitiesStates;
    }

    AZ::Outcome<void, FailedResult> SimulationEntitiesManager::SetEntityState(const AZStd::string& name, const EntityState& state)
    {
        if (auto outcome = IsWorldLoaded(); !outcome.IsSuccess())
        {
            return AZ::Failure(outcome.GetError());
        }

        const auto findIt = m_simulatedEntityToEntityIdMap.find(name);
        if (findIt == m_simulatedEntityToEntityIdMap.end())
        {
            AZ_Warning("SimulationInterfaces", false, "Entity %s not found", name.c_str());
            return AZ::Failure(FailedResult(simulation_interfaces::msg::Result::RESULT_NOT_FOUND, "Entity not found"));
        }

        const AZ::EntityId entityId = findIt->second;
        AZ_Assert(entityId.IsValid(), "EntityId is not valid");

        // get entity and all descendants
        AZStd::vector<AZ::EntityId> entityAndDescendants;
        AZ::TransformBus::EventResult(entityAndDescendants, entityId, &AZ::TransformBus::Events::GetEntityAndAllDescendants);
        SetEntitiesState(entityAndDescendants, state);
        return AZ::Success();
    }

    void SimulationEntitiesManager::SetEntitiesState(const AZStd::vector<AZ::EntityId>& entityAndDescendants, const EntityState& state)
    {
        if (entityAndDescendants.empty())
        {
            AZ_Error("SimulationInterfaces", false, "Entity and descendants list is empty");
            return;
        }
        const AZ::EntityId entityId = entityAndDescendants.front();
        AZ::EntityId parentEntityId = AZ::EntityId{ AZ::EntityId::InvalidEntityId };
        AZ::TransformBus::EventResult(parentEntityId, entityId, &AZ::TransformBus::Events::GetParentId);
        if (state.m_pose.IsOrthogonal())
        {
            // disable simulation for all entities
            for (const auto& descendant : entityAndDescendants)
            {
                AzPhysics::SimulatedBodyComponentRequestsBus::Event(descendant, &AzPhysics::SimulatedBodyComponentRequests::DisablePhysics);
            }
            if (parentEntityId.IsValid())
            {
                AZ::Transform parentTransform;
                AZ::TransformBus::EventResult(parentTransform, parentEntityId, &AZ::TransformBus::Events::GetWorldTM);
                float scale = 1.0f;
                AZ::TransformBus::EventResult(scale, entityId, &AZ::TransformBus::Events::GetLocalUniformScale);
                auto transformToSet = parentTransform.GetInverse() * state.m_pose;
                transformToSet.SetUniformScale(scale);
                AZ::TransformBus::Event(entityId, &AZ::TransformBus::Events::SetLocalTM, transformToSet);
            }
            else
            {
                AZ::TransformBus::Event(entityId, &AZ::TransformBus::Events::SetLocalTM, state.m_pose);
            }

            for (const auto& descendant : entityAndDescendants)
            {
                AzPhysics::SimulatedBodyComponentRequestsBus::Event(descendant, &AzPhysics::SimulatedBodyComponentRequests::EnablePhysics);
                Physics::RigidBodyRequestBus::Event(descendant, &Physics::RigidBodyRequests::SetAngularVelocity, AZ::Vector3::CreateZero());
                Physics::RigidBodyRequestBus::Event(descendant, &Physics::RigidBodyRequests::SetLinearVelocity, AZ::Vector3::CreateZero());
            }
        }

        if (!state.m_twistLinear.IsZero(AZ::Constants::FloatEpsilon) || !state.m_twistAngular.IsZero(AZ::Constants::FloatEpsilon))
        {
            // get rigid body
            AzPhysics::RigidBody* rigidBody = nullptr;
            Physics::RigidBodyRequestBus::EventResult(rigidBody, entityId, &Physics::RigidBodyRequests::GetRigidBody);
            if (rigidBody != nullptr)
            {
                SetRigidBodyVelocities(rigidBody, state);
            }
        }
    }

    void SimulationEntitiesManager::DeleteEntity(const AZStd::string& name, DeletionCompletedCb completedCb)
    {
        if (auto outcome = IsWorldLoaded(); !outcome.IsSuccess())
        {
            completedCb(AZ::Failure(outcome.GetError()));
            return;
        }

        const auto findIt = m_simulatedEntityToEntityIdMap.find(name);
        if (findIt == m_simulatedEntityToEntityIdMap.end())
        {
            completedCb(AZ::Failure(FailedResult(simulation_interfaces::msg::Result::RESULT_NOT_FOUND, "Entity not found")));
            return;
        }

        const AZ::EntityId entityId = findIt->second;
        AZ_Assert(entityId.IsValid(), "EntityId is not valid");
        // get entity
        AZ::Entity* entity = nullptr;
        AZ::ComponentApplicationBus::BroadcastResult(entity, &AZ::ComponentApplicationRequests::FindEntity, entityId);
        AZ_Assert(entity, "Entity is not available.");
        if (entity == nullptr)
        {
            AZ_Error("SimulationInterfaces", false, "Entity %s (%s) not found", name.c_str(), entityId.ToString().c_str());
            completedCb(AZ::Failure(FailedResult(simulation_interfaces::msg::Result::RESULT_NOT_FOUND, "Entity not found")));
            return;
        }
        // check if entity is spawned by this component
        const auto ticketId = entity->GetEntitySpawnTicketId();
        if (m_spawnedTickets.find(ticketId) != m_spawnedTickets.end())
        {
            // get spawner
            auto spawner = AZ::Interface<AzFramework::SpawnableEntitiesDefinition>::Get();
            AZ_Assert(spawner, "SpawnableEntitiesDefinition is not available.");
            // get ticket
            auto ticket = m_spawnedTickets[ticketId];
            // remove ticket
            AzFramework::DespawnAllEntitiesOptionalArgs optionalArgs;
            optionalArgs.m_completionCallback = [this, completedCb, name](AzFramework::EntitySpawnTicket::Id ticketId)
            {
                m_spawnedTickets.erase(ticketId);
                UnregisterSimulatedBody(name);
                completedCb(AZ::Success());
            };
            spawner->DespawnAllEntities(ticket, optionalArgs);
        }
        else
        {
            UnregisterSimulatedBody(name);
            const auto msg = AZStd::string::format(
                "Entity %s was not spawned by this component, wont delete it but name will be removed from registry immediately",
                name.c_str());
            completedCb(AZ::Failure(FailedResult(simulation_interfaces::msg::Result::RESULT_OPERATION_FAILED, msg)));
        }
    }

    void SimulationEntitiesManager::DeleteAllEntities(DeletionCompletedCb completedCb)
    {
        if (auto outcome = IsWorldLoaded(); !outcome.IsSuccess())
        {
            completedCb(AZ::Failure(outcome.GetError()));
            return;
        }

        if (m_spawnedTickets.empty())
        {
            // early return for empty scene
            completedCb(AZ::Success());
            return;
        }
        // clear collected data about simulated entities
        m_entityIdToSimulatedEntityMap.clear();
        m_simulatedEntityToEntityIdMap.clear();
        m_simulatedEntityToPrefabRoot.clear();
        m_entityIdToInitialState.clear();
        m_nameToEntityInfo.clear();
        m_categoryToNames.clear();

        for (auto m_spawnedTicket : m_spawnedTickets)
        {
            auto spawner = AZ::Interface<AzFramework::SpawnableEntitiesDefinition>::Get();
            AZ_Assert(spawner, "SpawnableEntitiesDefinition is not available.");
            // get ticket
            auto ticket = m_spawnedTickets[m_spawnedTicket.first];

            // despawn
            AzFramework::DespawnAllEntitiesOptionalArgs optionalArgs;
            optionalArgs.m_completionCallback = [this, completedCb](AzFramework::EntitySpawnTicket::Id ticketId)
            {
                m_spawnedTickets.erase(ticketId);
                if (completedCb && m_spawnedTickets.empty())
                {
                    completedCb(AZ::Success());
                }
            };
            spawner->DespawnAllEntities(ticket, optionalArgs);
        }
    }

    AZ::Outcome<SpawnableList, FailedResult> SimulationEntitiesManager::GetSpawnables()
    {
        AZStd::vector<Spawnable> spawnables;

        const auto enumCallback = [&spawnables](const AZ::Data::AssetId assetId, const AZ::Data::AssetInfo& assetInfo)
        {
            bool isSpawnable = false;
            AZ::Data::AssetCatalogRequestBus::BroadcastResult(
                isSpawnable, &AZ::Data::AssetCatalogRequests::DoesAssetIdMatchWildcardPattern, assetId, "*.spawnable");

            if (isSpawnable)
            {
                Spawnable spawnable;
                spawnable.m_uri = Utils::RelPathToUri(assetInfo.m_relativePath);
                spawnables.push_back(spawnable);
            }
        };

        AZ::Data::AssetCatalogRequestBus::Broadcast(&AZ::Data::AssetCatalogRequests::EnumerateAssets, nullptr, enumCallback, nullptr);
        return AZ::Success(spawnables);
    }

    void SimulationEntitiesManager::SpawnEntity(
        const AZStd::string& name,
        const AZStd::string& uri,
        const AZStd::string& entityNamespace,
        const AZ::Transform& initialPose,
        const bool allowRename,
        PreInsertionCb preinsertionCb,
        SpawnCompletedCb completedCb)
    {
        if (auto outcome = IsWorldLoaded(); !outcome.IsSuccess())
        {
            completedCb(AZ::Failure(outcome.GetError()));
            return;
        }
        if (!allowRename)
        {
            // If API user does not allow renaming, check if name is unique
            if (m_simulatedEntityToEntityIdMap.contains(name))
            {
                const auto msg = AZStd::string::format("Entity name %s is not unique", name.c_str());
                completedCb(AZ::Failure(FailedResult(simulation_interfaces::srv::SpawnEntity::Response::NAME_NOT_UNIQUE, msg)));
                return;
            }
            if (name.empty())
            {
                const auto msg = AZStd::string::format("Entity name is empty");
                completedCb(AZ::Failure(FailedResult(simulation_interfaces::srv::SpawnEntity::Response::NAME_INVALID, msg)));
                return;
            }
        }
        if (!initialPose.IsOrthogonal())
        {
            AZ_Warning("SimulationInterfaces", false, "Initial pose is not orthogonal");
            completedCb(
                AZ::Failure(FailedResult(
                    simulation_interfaces::srv::SpawnEntity::Response::INVALID_POSE, "Initial pose is not orthogonal"))); //  INVALID_POSE
            return;
        }

        // get rel path from uri
        const AZStd::string relPath = Utils::UriToRelPath(uri);

        // create spawnable
        AZ::Data::AssetId assetId;
        AZ::Data::AssetCatalogRequestBus::BroadcastResult(
            assetId,
            &AZ::Data::AssetCatalogRequestBus::Events::GetAssetIdByPath,
            relPath.c_str(),
            azrtti_typeid<AZ::Data::AssetData>(),
            false);
        AZ_Warning("SimulationInterfaces", assetId.IsValid(), "AssetId is not valid, relative path %s", relPath.c_str());

        auto spawner = AZ::Interface<AzFramework::SpawnableEntitiesDefinition>::Get();
        AZ_Assert(spawner, "SpawnableEntitiesDefinition is not available.");

        AZ::Data::Asset<AzFramework::Spawnable> spawnableAsset =
            AZ::Data::AssetManager::Instance().GetAsset<AzFramework::Spawnable>(assetId, AZ::Data::AssetLoadBehavior::NoLoad);
        if (!spawnableAsset)
        {
            const auto msg = AZStd::string::format("Spawnable asset %s not found", uri.c_str());
            completedCb(AZ::Failure(FailedResult(simulation_interfaces::msg::Result::RESULT_NOT_FOUND, msg)));
            return;
        }

        auto ticket = AzFramework::EntitySpawnTicket(spawnableAsset);
        AzFramework::SpawnAllEntitiesOptionalArgs optionalArgs;

        optionalArgs.m_preInsertionCallback = [this, initialPose, entityNamespace](auto id, auto view)
        {
            auto spawnData = m_spawnCompletedCallbacks.find(id);
            if (view.empty())
            {
                if (spawnData != m_spawnCompletedCallbacks.end())
                {
                    spawnData->second.m_preInsertionCb(
                        AZ::Failure(FailedResult(simulation_interfaces::msg::Result::RESULT_OPERATION_FAILED, "Spawned prefab is empty")));
                }
                return;
            }
            AZ::Entity* root = *view.begin();

            for (AZ::Entity* entity : view)
            {
                auto* ros2Frame = entity->FindComponent<ROS2::ROS2FrameComponent>();
                if (ros2Frame)
                {
                    ros2Frame->UpdateNamespaceConfiguration(entityNamespace, ROS2::NamespaceConfiguration::NamespaceStrategy::Custom);
                    break;
                }
            }
            auto* transformInterface = root->FindComponent<AzFramework::TransformComponent>();
            if (transformInterface)
            {
                transformInterface->SetWorldTM(initialPose);
            }
            // run preinsertion Callback if exists
            if (spawnData != m_spawnCompletedCallbacks.end())
            {
                spawnData->second.m_preInsertionCb(AZ::Success(view));
            }
        };

        optionalArgs.m_completionCallback =
            [this](AzFramework::EntitySpawnTicket::Id ticketId, AzFramework::SpawnableConstEntityContainerView view)
        {
            const AZ::Entity* root = *view.begin();
            AZ::EntityId trackingEntity{ AZ::EntityId::InvalidEntityId };

            // find first physical entity to set as tracking entity. If none of physical entity was found, set root as tracking entity
            // if spawned prefab has tracking tag on any of the entities, force tracking entity owning the tag
            for (const auto* entity : view)
            {
                // check if entity is physical entity == check if it is simulated body, if so get first occurrence
                AzPhysics::SimulatedBody* simBody = nullptr;
                AzPhysics::SimulatedBodyComponentRequestsBus::EventResult(
                    simBody, entity->GetId(), &AzPhysics::SimulatedBodyComponentRequests::GetSimulatedBody);
                if ((!trackingEntity.IsValid() && simBody != nullptr) &&
                    (simBody->GetNativeType() == RigidBody || simBody->GetNativeType() == RigidBodyStatic ||
                     simBody->GetNativeType() == ArticulationLink))
                {
                    trackingEntity = entity->GetId();
                    break;
                }
            }

            if (!trackingEntity.IsValid()) // physical entity was not found, assign root instead
            {
                trackingEntity = root->GetId();
            }

            auto spawnData = m_spawnCompletedCallbacks.find(ticketId);
            if (spawnData != m_spawnCompletedCallbacks.end())
            {
                auto finalNameOutcome = RegisterNewSimulatedBody(spawnData->second.m_userProposedName, trackingEntity);
                if (finalNameOutcome.IsSuccess())
                {
                    m_simulatedEntityToPrefabRoot[finalNameOutcome.GetValue()] = root->GetId();
                    spawnData->second.m_completedCb(AZ::Success(finalNameOutcome.GetValue()));
                }
                else
                {
                    spawnData->second.m_completedCb(AZ::Failure(FailedResult(
                        simulation_interfaces::msg::Result::RESULT_OPERATION_FAILED, "Failed to spawn/or register simulation entity")));
                }
                m_spawnCompletedCallbacks.erase(spawnData);
            }
        };

        spawner->SpawnAllEntities(ticket, optionalArgs);
        auto ticketId = ticket.GetId();
        SpawnCompletedCbData data;
        // to ensure that callbacks are valid
        data.m_completedCb = completedCb ? completedCb : SpawnCompletedCb{};
        data.m_preInsertionCb = preinsertionCb ? preinsertionCb : PreInsertionCb{};
        data.m_userProposedName = name;
        m_spawnCompletedCallbacks[ticketId] = data;
        m_spawnedTickets[ticketId] = ticket;
        AZ_Info("SimulationInterfaces", "Spawning uri %s with ticket id %d\n", uri.c_str(), ticketId);
    }

    AZStd::string SimulationEntitiesManager::GetSimulatedEntityName(AZ::EntityId entityId, const AZStd::string& proposedName) const
    {
        // Get O3DE entity name
        AZStd::string newName = proposedName;
        // check if name is not unique. If not, add Entity Name to name
        if (m_simulatedEntityToEntityIdMap.contains(newName))
        {
            AZStd::string entityName;
            AZ::ComponentApplicationBus::BroadcastResult(entityName, &AZ::ComponentApplicationRequests::GetEntityName, entityId);
            // name is not unique, add entityId to name
            newName = AZStd::string::format("%s_%s", newName.c_str(), entityName.c_str());
        }

        // check if name is still not unique, if not, add EntityId to name
        if (m_simulatedEntityToEntityIdMap.contains(newName))
        {
            newName = AZStd::string::format("%s_%s", newName.c_str(), entityId.ToString().c_str());
        }
        return newName;
    }

    AZ::Outcome<void, FailedResult> SimulationEntitiesManager::ResetAllEntitiesToInitialState()
    {
        if (auto outcome = IsWorldLoaded(); !outcome.IsSuccess())
        {
            return AZ::Failure(outcome.GetError());
        }

        for (const auto& [entityId, initialState] : m_entityIdToInitialState)
        {
            AZStd::vector<AZ::EntityId> entityAndDescendants;
            AZ::TransformBus::EventResult(entityAndDescendants, entityId, &AZ::TransformBus::Events::GetEntityAndAllDescendants);

            SetEntitiesState(entityAndDescendants, initialState);
        }
        return AZ::Success();
    }

    AZ::Outcome<void, FailedResult> SimulationEntitiesManager::IsWorldLoaded()
    {
        bool canOperate = false;
        SimulationManagerRequestBus::BroadcastResult(canOperate, &SimulationManagerRequests::EntitiesOperationsPossible);
        if (!canOperate)
        {
            return AZ::Failure(FailedResult(
                simulation_interfaces::msg::Result::RESULT_OPERATION_FAILED,
                "Simulator needs to have loaded world to allow entities manipulation"));
        }
        return AZ::Success();
    }

    AZ::Outcome<void, SimulationInterfaces::FailedResult> SimulationEntitiesManager::SetEntityInfo(
        const AZStd::string& name, const EntityInfo& info)
    {
        if (!m_simulatedEntityToEntityIdMap.contains(name))
        {
            return AZ::Failure(SimulationInterfaces::FailedResult(
                simulation_interfaces::msg::Result::RESULT_OPERATION_FAILED,
                AZStd::string::format("Entity with given name \"%s\" doesn't exists", name.c_str())));
        }
        // check if entity with given name has entity info assigned, clear it if needed
        RemoveEntityInfoIfNeeded(name);
        // add to cache storing entityInfo by name. it is ensured that name is deleted if it already exists
        m_nameToEntityInfo.insert({ name, info });
        // assign entity name to category
        // if set is not created yet, create it
        if (!m_categoryToNames.contains(info.m_category))
        {
            m_categoryToNames.emplace(info.m_category, AZStd::unordered_set<AZStd::string>{});
        }
        m_categoryToNames.at(info.m_category).insert(name);

        return AZ::Success();
    }

    AZ::Outcome<EntityInfo, SimulationInterfaces::FailedResult> SimulationEntitiesManager::GetEntityInfo(const AZStd::string& name)
    {
        if (!m_simulatedEntityToEntityIdMap.contains(name))
        {
            return AZ::Failure(SimulationInterfaces::FailedResult(
                simulation_interfaces::msg::Result::RESULT_OPERATION_FAILED,
                AZStd::string::format("Entity with given name \"%s\" doesn't exists", name.c_str())));
        }

        auto findIt = m_nameToEntityInfo.find(name);
        if (findIt == m_nameToEntityInfo.end())
        {
            const auto msg = AZStd::string::format("Entity with given name \"%s\" doesn't have assigned EntityInfo", name.c_str());
            AZ_Warning("SpawnableSceneProviderSystemComponent", false, msg.c_str());
            return AZ::Failure(SimulationInterfaces::FailedResult(simulation_interfaces::msg::Result::RESULT_OPERATION_FAILED, msg));
        }
        return AZ::Success(findIt->second);
    }

    void SimulationEntitiesManager::RemoveEntityInfoIfNeeded(const AZStd::string& name)
    {
        // if deleted entity had assigned entity info, remove it
        if (m_nameToEntityInfo.contains(name))
        {
            auto info = m_nameToEntityInfo.at(name);
            m_nameToEntityInfo.erase(name);
            // if entity Info is added, entity name is added to category to name map, lack of this entity in this map is clearly a bug
            AZ_Assert(m_categoryToNames.contains(info.m_category), "Failed to get entities with category %d", info.m_category);
            m_categoryToNames.at(info.m_category).erase(name);
        }
    }

    AZ::Outcome<Bounds, FailedResult> SimulationEntitiesManager::GetEntityBounds(const AZStd::string& name)
    {
        if (!m_simulatedEntityToEntityIdMap.contains(name))
        {
            return AZ::Failure(SimulationInterfaces::FailedResult(
                simulation_interfaces::msg::Result::RESULT_OPERATION_FAILED,
                AZStd::string::format("Entity with given name \"%s\" doesn't exists", name.c_str())));
        }
        auto simulatedBodyOutcome = Utils::GetSimulatedBody(m_simulatedEntityToEntityIdMap.at(name));
        if (!simulatedBodyOutcome.IsSuccess())
        {
            return AZ::Success(Bounds{ 0, {} });
        }
        auto rigidBody = azdynamic_cast<AzPhysics::RigidBody*>(simulatedBodyOutcome.GetValue());
        if (!rigidBody)
        {
            return AZ::Success(Bounds{ 0, {} });
        }
        if (rigidBody->GetShapeCount() == 0)
        {
            return AZ::Failure(FailedResult(
                simulation_interfaces::msg::Result::RESULT_OPERATION_FAILED, "Entity doesn't have colliders/boundss to return"));
        }
        AZ_Warning(
            "Simulation Interfaces",
            rigidBody->GetShapeCount() == 1,
            "Entity Bounds in simulation interfaces doesn't support multiple shapes, only first one will be taken ");
        auto shape = rigidBody->GetShape(0);
        auto boundsOutput = Utils::ConvertPhysicalShapeToBounds(shape, m_simulatedEntityToEntityIdMap.at(name));
        if (boundsOutput.IsSuccess())
        {
            return AZ::Success(boundsOutput.GetValue());
        }

        return AZ::Failure(FailedResult(simulation_interfaces::msg::Result::RESULT_OPERATION_FAILED, boundsOutput.GetError()));
    }

    AZ::Outcome<AZ::EntityId, FailedResult> SimulationEntitiesManager::GetEntityId(const AZStd::string& name)
    {
        if (!m_simulatedEntityToEntityIdMap.contains(name))
        {
            return AZ::Failure(SimulationInterfaces::FailedResult(
                simulation_interfaces::msg::Result::RESULT_OPERATION_FAILED,
                AZStd::string::format("Entity with given name \"%s\" doesn't exists", name.c_str())));
        }
        return AZ::Success(m_simulatedEntityToEntityIdMap.at(name));
    }

    AZ::Outcome<AZ::EntityId, FailedResult> SimulationEntitiesManager::GetEntityRoot(const AZStd::string& name)
    {
        if (!m_simulatedEntityToPrefabRoot.contains(name))
        {
            return AZ::Failure(SimulationInterfaces::FailedResult(
                simulation_interfaces::msg::Result::RESULT_OPERATION_FAILED,
                AZStd::string::format("Entity with given name \"%s\" doesn't exists in available cache of prefab roots", name.c_str())));
        }
        return AZ::Success(m_simulatedEntityToPrefabRoot.at(name));
    }

} // namespace SimulationInterfaces
