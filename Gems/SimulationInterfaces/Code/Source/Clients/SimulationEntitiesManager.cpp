/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "SimulationEntitiesManager.h"

#include <SimulationInterfaces/SimulationInterfacesTypeIds.h>

#include "Clients/SimulationFeaturesAggregator.h"
#include "CommonUtilities.h"
#include "ConsoleCommands.inl"
#include "SimulationInterfaces/SimulationFeatures.h"
#include "SimulationInterfaces/SimulationFeaturesAggregatorRequestBus.h"
#include <AzCore/Asset/AssetManager.h>
#include <AzCore/Asset/AssetManagerBus.h>
#include <AzCore/Component/ComponentApplicationBus.h>
#include <AzCore/Component/TransformBus.h>
#include <AzCore/Console/IConsole.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/std/containers/vector.h>
#include <AzCore/std/string/regex.h>
#include <AzFramework/Components/TransformComponent.h>
#include <AzFramework/Physics/PhysicsSystem.h>
#include <AzFramework/Physics/RigidBodyBus.h>
#include <AzFramework/Physics/SimulatedBodies/RigidBody.h>
#include <AzFramework/Spawnable/Spawnable.h>
#include <AzFramework/Spawnable/SpawnableEntitiesInterface.h>

namespace SimulationInterfaces
{
    void SetRigidBodyVelocities(AzPhysics::RigidBody* rigidBody, const EntityState& state)
    {
        if (!state.m_twist_angular.IsClose(AZ::Vector3::CreateZero(), AZ::Constants::FloatEpsilon))
        {
            // get transform
            AZ::Vector3 angularVelWorld = rigidBody->GetTransform().TransformVector(state.m_twist_angular);
            rigidBody->SetAngularVelocity(angularVelWorld);
        }

        if (!state.m_twist_linear.IsClose(AZ::Vector3::CreateZero(), AZ::Constants::FloatEpsilon))
        {
            // get transform
            AZ::Vector3 linearVelWorld = rigidBody->GetTransform().TransformVector(state.m_twist_linear);
            rigidBody->SetAngularVelocity(linearVelWorld);
        }
    }

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
        required.push_back(AZ_CRC_CE("SimulationFeaturesAggregator"));
    }

    void SimulationEntitiesManager::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
        dependent.push_back(AZ_CRC_CE("PhysicsService"));
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

    void SimulationEntitiesManager::Init()
    {
    }

    AzPhysics::Scene* GetSceneHelper(AzPhysics::SceneHandle sceneHandle)
    {
        AzPhysics::SystemInterface* physicsSystem = AZ::Interface<AzPhysics::SystemInterface>::Get();
        AZ_Assert(physicsSystem, "Physics system is not available.");
        AzPhysics::Scene* scene = physicsSystem->GetScene(sceneHandle);
        return scene;
    }

    void SimulationEntitiesManager::Activate()
    {
        m_simulationBodyAddedHandler = AzPhysics::SceneEvents::OnSimulationBodyAdded::Handler(
            [this](AzPhysics::SceneHandle sceneHandle, AzPhysics::SimulatedBodyHandle bodyHandle)
            {
                auto* scene = GetSceneHelper(sceneHandle);
                if (scene == nullptr)
                {
                    return;
                }
                auto* body = scene->GetSimulatedBodyFromHandle(bodyHandle);
                AZ_Assert(body, "Simulated body is not available.");
                auto* rigidBody = azdynamic_cast<AzPhysics::RigidBody*>(body);
                if (rigidBody != nullptr)
                {
                    [[maybe_unused]] auto shapeCount = rigidBody->GetShapeCount();
                    AZ_Warning(
                        "SimulationInterfaces",
                        shapeCount > 0,
                        "Entity %s has no collider shapes, it won't be available by bound search",
                        rigidBody->GetEntityId().ToString().c_str());
                }
                const AZ::EntityId entityId = body->GetEntityId();
                AZ::Entity* entity = nullptr;
                AZ::ComponentApplicationBus::BroadcastResult(entity, &AZ::ComponentApplicationRequests::FindEntity, entityId);
                AZ_Assert(entity, "Entity is not available.");
                // check if entity is not spawned by this component
                const auto ticketId = entity->GetEntitySpawnTicketId();
                AZStd::string proposedName{};
                // check if ticket is in the unregistered list

                auto spawnData = m_spawnCompletedCallbacks.find(ticketId);
                if (spawnData != m_spawnCompletedCallbacks.end())
                {
                    proposedName = spawnData->second.m_userProposedName;
                }

                const AZStd::string registeredName = this->AddSimulatedEntity(entityId, proposedName);
                // call the callback
                if (spawnData != m_spawnCompletedCallbacks.end())
                {
                    // call and remove the callback
                    spawnData->second.m_completedCb(AZ::Success(registeredName));
                    m_spawnCompletedCallbacks.erase(spawnData);
                }
            });
        m_simulationBodyRemovedHandler = AzPhysics::SceneEvents::OnSimulationBodyRemoved::Handler(
            [this](AzPhysics::SceneHandle sceneHandle, AzPhysics::SimulatedBodyHandle bodyHandle)
            {
                auto* scene = GetSceneHelper(sceneHandle);
                if (scene == nullptr)
                {
                    return;
                }
                const auto* body = scene->GetSimulatedBodyFromHandle(bodyHandle);
                AZ_Assert(body, "Simulated body is not available.");
                const AZ::EntityId entityId = body->GetEntityId();
                // remove simulated entity
                this->RemoveSimulatedEntity(entityId);
            });

        m_sceneAddedHandler = AzPhysics::SystemEvents::OnSceneAddedEvent::Handler(
            [this](AzPhysics::SceneHandle sceneHandle)
            {
                AZ_Warning("SimulationInterfaces", m_physicsScenesHandle == AzPhysics::InvalidSceneHandle, "Hmm, we already have a scene");
                auto* scene = GetSceneHelper(sceneHandle);
                AZ_Assert(scene, "Scene is not available.");
                if (scene == nullptr)
                {
                    return;
                }
                scene->RegisterSimulationBodyAddedHandler(m_simulationBodyAddedHandler);
                scene->RegisterSimulationBodyRemovedHandler(m_simulationBodyRemovedHandler);

                AZ_Printf("SimulationInterfaces", "Registered simulation body added handler\n");
                m_physicsScenesHandle = sceneHandle;
            });
        m_sceneRemovedHandler = AzPhysics::SystemEvents::OnSceneRemovedEvent::Handler(
            [this](AzPhysics::SceneHandle sceneHandle)
            {
                if (m_physicsScenesHandle == sceneHandle)
                {
                    m_entityIdToSimulatedEntityMap.clear();
                    m_simulatedEntityToEntityIdMap.clear();
                    m_simulationBodyAddedHandler.Disconnect();
                    m_simulationBodyRemovedHandler.Disconnect();
                    m_physicsScenesHandle = AzPhysics::InvalidSceneHandle;
                }
            });
        AzPhysics::SystemInterface* physicsSystem = AZ::Interface<AzPhysics::SystemInterface>::Get();
        if (physicsSystem)
        {
            physicsSystem->RegisterSceneAddedEvent(m_sceneAddedHandler);
            physicsSystem->RegisterSceneRemovedEvent(m_sceneRemovedHandler);
            SimulationEntityManagerRequestBus::Handler::BusConnect();
        }

        SimulationFeaturesAggregatorRequestBus::Broadcast(
            &SimulationFeaturesAggregatorRequests::AddSimulationFeatures,
            AZStd::unordered_set<SimulationFeatures>{ SimulationFeatures::ENTITY_TAGS,
                                                      SimulationFeatures::ENTITY_BOUNDS_BOX,
                                                      SimulationFeatures::ENTITY_BOUNDS_CONVEX,
                                                      SimulationFeatures::ENTITY_CATEGORIES,
                                                      SimulationFeatures::ENTITY_STATE_GETTING,
                                                      SimulationFeatures::ENTITY_STATE_SETTING,
                                                      SimulationFeatures::DELETING,
                                                      SimulationFeatures::SPAWNABLES,
                                                      SimulationFeatures::SPAWNING });
    }

    void SimulationEntitiesManager::Deactivate()
    {
        SimulationEntityManagerRequestBus::Handler::BusDisconnect();
        if (m_simulationBodyAddedHandler.IsConnected())
        {
            m_simulationBodyAddedHandler.Disconnect();
        }
        if (m_simulationBodyRemovedHandler.IsConnected())
        {
            m_simulationBodyRemovedHandler.Disconnect();
        }
        m_physicsScenesHandle = AzPhysics::InvalidSceneHandle;
        if (m_sceneAddedHandler.IsConnected())
        {
            m_sceneAddedHandler.Disconnect();
        }
        if (m_sceneAddedHandler.IsConnected())
        {
            m_sceneAddedHandler.Disconnect();
        }
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
        AZ_Printf("SimulationInterfaces", "Registered entity %s\n", simulatedEntityName.c_str());
        return simulatedEntityName;
    }

    void SimulationEntitiesManager::RemoveSimulatedEntity(AZ::EntityId entityId)
    {
        auto findIt = m_entityIdToSimulatedEntityMap.find(entityId);
        if (findIt != m_entityIdToSimulatedEntityMap.end())
        {
            const auto& simulatedEntityName = findIt->second;
            m_entityIdToSimulatedEntityMap.erase(findIt);
            m_simulatedEntityToEntityIdMap.erase(simulatedEntityName);
        }
    }

    AZ::Outcome<EntityNameList, FailedResult> SimulationEntitiesManager::GetEntities(const EntityFilters& filter)
    {
        if (!filter.m_tags_filter.m_tags.empty())
        {
            AZ_Warning("SimulationInterfaces", false, "Tags filter is not implemented yet");
            return AZ::Failure(FailedResult(ErrorCode::RESULT_FEATURE_UNSUPPORTED, "Tags filter is not implemented yet"));
        }

        const bool reFilter = !filter.m_filter.empty();
        const bool shapeCastFilter = filter.m_bounds_shape != nullptr;

        AZStd::vector<AZStd::string> entities;
        if (!shapeCastFilter)
        {
            // get all entities from the map
            entities.reserve(m_entityIdToSimulatedEntityMap.size());
            AZStd::transform(
                m_entityIdToSimulatedEntityMap.begin(),
                m_entityIdToSimulatedEntityMap.end(),
                AZStd::back_inserter(entities),
                [](const auto& pair)
                {
                    return pair.second;
                });
        }
        else
        {
            auto* sceneInterface = AZ::Interface<AzPhysics::SceneInterface>::Get();
            AZ_Assert(sceneInterface, "Physics scene interface is not available.");

            if (m_physicsScenesHandle == AzPhysics::InvalidSceneHandle)
            {
                return AZ::Failure(FailedResult(ErrorCode::RESULT_OPERATION_FAILED, "Physics scene interface is not available."));
            }

            AzPhysics::OverlapRequest request;
            request.m_shapeConfiguration = filter.m_bounds_shape;
            request.m_pose = filter.m_bounds_pose;
            request.m_maxResults = AZStd::numeric_limits<AZ::u32>::max();

            AzPhysics::SceneQueryHits result = sceneInterface->QueryScene(m_physicsScenesHandle, &request);
            for (const auto& hit : result.m_hits)
            {
                const AZ::EntityId entityId = hit.m_entityId;
                auto findIt = m_entityIdToSimulatedEntityMap.find(entityId);
                if (findIt != m_entityIdToSimulatedEntityMap.end())
                {
                    entities.push_back(findIt->second);
                }
            }
        }
        if (reFilter)
        {
            const AZStd::vector<AZStd::string> prefilteredEntities = AZStd::move(entities);
            entities.clear();
            const AZStd::regex regex(filter.m_filter);
            if (!regex.Valid())
            {
                AZ_Warning("SimulationInterfaces", false, "Invalid regex filter");
                return AZ::Failure(FailedResult(ErrorCode::RESULT_NOT_FOUND, "Invalid regex filter"));
            }
            AZStd::ranges::copy_if(
                prefilteredEntities,
                AZStd::back_inserter(entities),
                [&regex](const AZStd::string& entityName)
                {
                    return AZStd::regex_search(entityName, regex);
                });
        }
        return AZ::Success(entities);
    }

    AZ::Outcome<EntityState, FailedResult> SimulationEntitiesManager::GetEntityState(const AZStd::string& name)
    {
        const auto findIt = m_simulatedEntityToEntityIdMap.find(name);
        if (findIt == m_simulatedEntityToEntityIdMap.end())
        {
            AZ_Warning("SimulationInterfaces", findIt != m_simulatedEntityToEntityIdMap.end(), "Entity %s not found", name.c_str());
            return AZ::Failure(FailedResult(ErrorCode::RESULT_NOT_FOUND, "Entity not found"));
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
        entityState.m_twist_linear = entityTransformInv.TransformVector(linearVelocity);
        entityState.m_twist_angular = entityTransformInv.TransformVector(angularVelocity);
        return AZ::Success(entityState);
    }

    AZ::Outcome<MultipleEntitiesStates, FailedResult> SimulationEntitiesManager::GetEntitiesStates(const EntityFilters& filter)
    {
        if (!filter.m_tags_filter.m_tags.empty())
        {
            AZ_Warning("SimulationInterfaces", false, "Tags filter is not implemented yet");
            return AZ::Failure(FailedResult(ErrorCode::RESULT_FEATURE_UNSUPPORTED, "Tags filter is not implemented yet"));
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
        const auto findIt = m_simulatedEntityToEntityIdMap.find(name);
        if (findIt == m_simulatedEntityToEntityIdMap.end())
        {
            AZ_Warning("SimulationInterfaces", false, "Entity %s not found", name.c_str());
            return AZ::Failure(FailedResult(ErrorCode::RESULT_NOT_FOUND, "Entity not found"));
        }

        const AZ::EntityId entityId = findIt->second;
        AZ_Assert(entityId.IsValid(), "EntityId is not valid");

        // get entity and all descendants
        AZStd::vector<AZ::EntityId> entityAndDescendants;
        AZ::TransformBus::EventResult(entityAndDescendants, entityId, &AZ::TransformBus::Events::GetEntityAndAllDescendants);

        if (state.m_pose.IsOrthogonal())
        {
            // disable simulation for all entities
            AZStd::map<AZ::EntityId, AZ::Transform> entityTransforms;
            for (const auto& descendant : entityAndDescendants)
            {
                // get name
                AZStd::string entityName = "Unknown";
                AZ::ComponentApplicationBus::BroadcastResult(entityName, &AZ::ComponentApplicationRequests::GetEntityName, descendant);
                AZ_Printf("SimulationInterfaces", "Disable physics for entity %s\n", entityName.c_str());
                Physics::RigidBodyRequestBus::Event(descendant, &Physics::RigidBodyRequests::DisablePhysics);
            }

            AZ::TransformBus::Event(entityId, &AZ::TransformBus::Events::SetLocalTM, state.m_pose);

            for (const auto& descendant : entityAndDescendants)
            {
                Physics::RigidBodyRequestBus::Event(descendant, &Physics::RigidBodyRequests::EnablePhysics);
                Physics::RigidBodyRequestBus::Event(descendant, &Physics::RigidBodyRequests::SetAngularVelocity, AZ::Vector3::CreateZero());
                Physics::RigidBodyRequestBus::Event(descendant, &Physics::RigidBodyRequests::SetLinearVelocity, AZ::Vector3::CreateZero());
            }
        }

        if (!state.m_twist_linear.IsZero(AZ::Constants::FloatEpsilon) || !state.m_twist_angular.IsZero(AZ::Constants::FloatEpsilon))
        {
            // get rigid body
            AzPhysics::RigidBody* rigidBody = nullptr;
            Physics::RigidBodyRequestBus::EventResult(rigidBody, entityId, &Physics::RigidBodyRequests::GetRigidBody);
            if (rigidBody != nullptr)
            {
                SetRigidBodyVelocities(rigidBody, state);
            }
        }
        return AZ::Success();
    }

    void SimulationEntitiesManager::DeleteEntity(const AZStd::string& name, DeletionCompletedCb completedCb)
    {
        const auto findIt = m_simulatedEntityToEntityIdMap.find(name);

        if (findIt == m_simulatedEntityToEntityIdMap.end())
        {
            completedCb(AZ::Failure(FailedResult(ErrorCode::RESULT_NOT_FOUND, "Entity not found")));
        }

        const AZ::EntityId entityId = findIt->second;
        AZ_Assert(entityId.IsValid(), "EntityId is not valid");
        // get entity
        AZ::Entity* entity = nullptr;
        AZ::ComponentApplicationBus::BroadcastResult(entity, &AZ::ComponentApplicationRequests::FindEntity, entityId);
        AZ_Assert(entity, "Entity is not available.");
        // check if entity is spawned by this component
        const auto ticketId = entity->GetEntitySpawnTicketId();
        if (m_spawnedTickets.find(ticketId) != m_spawnedTickets.end())
        {
            // remove the ticket
            // m_spawnedTickets.erase(ticketId);
            /// get spawner
            auto spawner = AZ::Interface<AzFramework::SpawnableEntitiesDefinition>::Get();
            AZ_Assert(spawner, "SpawnableEntitiesDefinition is not available.");
            // get ticket
            auto ticket = m_spawnedTickets[ticketId];
            // remove ticket
            AzFramework::DespawnAllEntitiesOptionalArgs optionalArgs;
            optionalArgs.m_completionCallback = [this, completedCb](AzFramework::EntitySpawnTicket::Id ticketId)
            {
                m_spawnedTickets.erase(ticketId);
                completedCb(AZ::Success());
            };
            spawner->DespawnAllEntities(ticket, optionalArgs);
        }
        else
        {
            const auto msg = AZStd::string::format("Entity %s was not spawned by this component, wont delete it", name.c_str());
            completedCb(AZ::Failure(FailedResult(ErrorCode::RESULT_OPERATION_FAILED, msg)));
        }
#ifdef POTENTIALY_UNSAFE
        if (findIt != m_simulatedEntityToEntityIdMap.end())
        {
            const AZ::EntityId entityId = findIt->second;
            AZ_Assert(entityId.IsValid(), "EntityId is not valid");
            // get all descendants
            AZStd::vector<AZ::EntityId> entityAndDescendants;
            AZ::TransformBus::EventResult(entityAndDescendants, entityId, &AZ::TransformBus::Events::GetEntityAndAllDescendants);
            for (const auto& descendant : entityAndDescendants)
            {
                // I am not sure if this is the safe way to delete an entity
                AZ::ComponentApplicationBus::Broadcast(&AZ::ComponentApplicationRequests::DeleteEntity, descendant);
            }

            return true;
        }
#endif
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
        SpawnCompletedCb completedCb)
    {
        // get rel path from uri
        const AZStd::string relPath = Utils::UriToRelPath(uri);

        // create spawnnable
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
            completedCb(AZ::Failure(FailedResult(ErrorCode::RESULT_NOT_FOUND, msg)));
            return;
        }

        auto ticket = AzFramework::EntitySpawnTicket(spawnableAsset);
        AzFramework::SpawnAllEntitiesOptionalArgs optionalArgs;

        optionalArgs.m_preInsertionCallback = [initialPose, entityNamespace, name](auto id, auto view)
        {
            if (view.empty())
            {
                return;
            }
            const AZ::Entity* root = *view.begin();

            // change names for all entites
            for (auto* entity : view)
            {
                AZStd::string entityName = AZStd::string::format("%s_%s", name.c_str(), entity->GetName().c_str());
                entity->SetName(entityName);
            }

            auto* transformInterface = root->FindComponent<AzFramework::TransformComponent>();
            if (transformInterface)
            {
                transformInterface->SetWorldTM(initialPose);
            }

            if (!entityNamespace.empty())
            {
                // TODO: Mpelka set ROS 2 namespace here
                AZ_Error("SimulationInterfaces", false, "ROS 2 namespace is not implemented yet in spawning");
            }
        };
        optionalArgs.m_completionCallback =
            [this, uri](AzFramework::EntitySpawnTicket::Id ticketId, AzFramework::SpawnableConstEntityContainerView view)
        {
            // at this point the entities are spawned and should be registered in simulation interface and callback should be called
            // if that is not a case, it means that the AZFramework::Physics::OnSimulationBodyAdded event was not called.
            // That means the prefab has no physics component or the physics component is not enabled - we need to call the callback here
            // and return the error.
            auto spawnData = m_spawnCompletedCallbacks.find(ticketId);
            if (spawnData != m_spawnCompletedCallbacks.end())
            {
                // call and remove the callback
                const auto msg = AZStd::string::format(
                    "Entity %s (uri : %s) was not registered in simulation interface - "
                    "no physics component or physics component is not enabled in source prefab.\n"
                    "Entity will be in simulation, but not available in simulation interface.\n"
                    "Please add some physics component (at least one static rigid body component) to the prefab.\n"
                    "Technically, it is a memory leak.\n",
                    spawnData->second.m_userProposedName.c_str(),
                    uri.c_str());
                AZ_Error("SimulationInterfaces", false, msg.c_str());
                spawnData->second.m_completedCb(msg);
                m_spawnCompletedCallbacks.erase(spawnData);
            }
        };

        spawner->SpawnAllEntities(ticket, optionalArgs);

        auto ticketId = ticket.GetId();
        AZ_Printf("SimulationInterfaces", "Spawning uri %s with ticket id %d\n", uri.c_str(), ticketId);

        SpawnCompletedCbData data;
        data.m_userProposedName = name;
        data.m_completedCb = completedCb;
        m_spawnCompletedCallbacks[ticketId] = data;
        m_spawnedTickets[ticketId] = ticket;
    }

    AZStd::string SimulationEntitiesManager::GetSimulatedEntityName(AZ::EntityId entityId, const AZStd::string& proposedName) const
    {
        // Get O3DE entity name
        AZStd::string entityName = proposedName;
        if (entityName.empty())
        {
            AZ::ComponentApplicationBus::BroadcastResult(entityName, &AZ::ComponentApplicationRequests::GetEntityName, entityId);
        }
        // Generate unique simulated entity name
        AZStd::string simulatedEntityName = entityName;
        // check if name is unique
        auto otherEntityIt = m_simulatedEntityToEntityIdMap.find(simulatedEntityName);
        if (otherEntityIt != m_simulatedEntityToEntityIdMap.end())
        {
            // name is not unique, add entityId to name
            simulatedEntityName = AZStd::string::format("%s_%s", entityName.c_str(), entityId.ToString().c_str());
        }
        return simulatedEntityName;
    }
} // namespace SimulationInterfaces
