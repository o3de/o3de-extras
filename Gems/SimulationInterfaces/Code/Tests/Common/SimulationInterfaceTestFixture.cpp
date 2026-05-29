/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "SimulationInterfaceTestFixture.h"

#include <AzCore/Asset/AssetManagerBus.h>
#include <AzCore/Component/EntityId.h>
#include <AzCore/Component/TransformBus.h>
#include <SimulationInterfaces/SimulationEntityManagerRequestBus.h>
#include <gtest/gtest.h>

namespace UnitTest
{
    void SimulationInterfaceTestFixture::AddAsset(const AZStd::string& assetPath)
    {
        AZ::Data::AssetInfo info;
        info.m_relativePath = assetPath;
        info.m_sizeBytes = 1;
        AZ::Data::AssetId id = AZ::Data::AssetId(AZ::Uuid::CreateRandom());
        AZ::Data::AssetCatalogRequestBus::Broadcast(&AZ::Data::AssetCatalogRequestBus::Events::RegisterAsset, id, info);
        m_registeredAssets.insert(id);
    }

    AzPhysics::SceneHandle SimulationInterfaceTestFixture::GetDefaultSceneHandle() const
    {
        return m_testSceneHandle;
    }

    AZ::EntityId SimulationInterfaceTestFixture::CreateEntityWithStaticBodyComponent(
        const AZStd::string& entityName, const AZ::Transform& transform)
    {
        AZStd::unique_ptr<AZ::Entity> entity = AZStd::make_unique<AZ::Entity>(entityName.c_str());
        auto* transformComponent = entity->CreateComponent(AZ::TransformComponentTypeId);
        AZ_Assert(transformComponent, "Failed to create TransformComponent");
        auto* transformInterface = azrtti_cast<AZ::TransformInterface*>(transformComponent);
        AZ_Assert(transformInterface, "Failed to get TransformInterface");
        transformInterface->SetWorldTM(transform);
        entity->CreateComponent(AZ::Uuid(PhysXRigidBodyComponentTypeId));
        entity->CreateComponent(AZ::Uuid(PhysXShapeColliderComponentTypeId));
        entity->CreateComponent(AZ::Uuid(SphereShapeComponentTypeId));
        entity->Init();
        entity->SetEntityActive(true);
        entity->ApplyEffectiveActiveState();
        AZ_Assert(entity->GetState() == AZ::Entity::State::Active, "Entity is not active");

        // register entity
        AZ::Outcome<AZStd::string, SimulationInterfaces::FailedResult> output;
        SimulationInterfaces::SimulationEntityManagerRequestBus::BroadcastResult(
            output, &SimulationInterfaces::SimulationEntityManagerRequests::RegisterNewSimulatedBody, entityName, entity->GetId());
        // function return type is different than void, so ASSERT_TRUE cannot be used. Expect_true and early return is added to mimic assert
        // behaviour
        EXPECT_TRUE(output.IsSuccess()) << "Failed to register entity to simulation_interfaces, "
                                        << output.GetError().m_errorString.c_str();
        if (!output.IsSuccess())
        {
            return AZ::EntityId{ AZ::EntityId::InvalidEntityId };
        }
        entity->SetName(output.GetValue()); // name in simulation_interfaces registry could be change to ensure uniqueness. Apply new name
                                            // to the created entity

        auto id = entity->GetId();
        m_entities.emplace(AZStd::make_pair(id, AZStd::move(entity)));

        return id;
    }

    void SimulationInterfaceTestFixture::ClearEntities()
    {
        for (auto& entity : m_entities)
        {
            AZ::Outcome<void, SimulationInterfaces::FailedResult> output;
            SimulationInterfaces::SimulationEntityManagerRequestBus::BroadcastResult(
                output, &SimulationInterfaces::SimulationEntityManagerRequests::UnregisterSimulatedBody, entity.second->GetName());
            AZ_Assert(
                output.IsSuccess(), "Failed to unregister entity from simulation_interfaces, %s", output.GetError().m_errorString.c_str());
            entity.second->SetEntityActive(false);
            entity.second->ApplyEffectiveActiveState();
        }
        m_entities.clear();
    }

    void SimulationInterfaceTestFixture::DeleteEntity(const AZ::EntityId& entityId)
    {
        auto findIt = m_entities.find(entityId);
        if (findIt != m_entities.end())
        {
            AZ::Outcome<void, SimulationInterfaces::FailedResult> output;
            SimulationInterfaces::SimulationEntityManagerRequestBus::BroadcastResult(
                output, &SimulationInterfaces::SimulationEntityManagerRequests::UnregisterSimulatedBody, findIt->second->GetName());
            AZ_Assert(
                output.IsSuccess(), "Failed to unregister entity from simulation_interfaces, %s", output.GetError().m_errorString.c_str());
            findIt->second->SetEntityActive(false);
            findIt->second->ApplyEffectiveActiveState();
            m_entities.erase(findIt);
        }
    }

    void SimulationInterfaceTestFixture::SetUp()
    {
        if (auto* physicsSystem = AZ::Interface<AzPhysics::SystemInterface>::Get())
        {
            AzPhysics::SceneConfiguration sceneConfiguration = physicsSystem->GetDefaultSceneConfiguration();
            sceneConfiguration.m_sceneName = AzPhysics::DefaultPhysicsSceneName;
            m_testSceneHandle = physicsSystem->AddScene(sceneConfiguration);
            m_defaultScene = physicsSystem->GetScene(m_testSceneHandle);
        }

        Physics::DefaultWorldBus::Handler::BusConnect();
    }

    void SimulationInterfaceTestFixture::TearDown()
    {
        ClearEntities();
        Physics::DefaultWorldBus::Handler::BusDisconnect();
        m_defaultScene = nullptr;

        // Clean up the Test scene
        if (auto* physicsSystem = AZ::Interface<AzPhysics::SystemInterface>::Get())
        {
            physicsSystem->RemoveScene(m_testSceneHandle);
        }
        m_testSceneHandle = AzPhysics::InvalidSceneHandle;

        for (const auto& id : m_registeredAssets)
        {
            AZ::Data::AssetCatalogRequestBus::Broadcast(&AZ::Data::AssetCatalogRequestBus::Events::UnregisterAsset, id);
        }
        m_registeredAssets.clear();
    }

    void SimulationInterfaceTestFixture::StepPhysics(int numSteps)
    {
        auto* physicsSystem = AZ::Interface<AzPhysics::SystemInterface>::Get();
        AZ_Assert(physicsSystem, "Failed to get physics system interface");
        for (int i = 0; i < numSteps; i++)
        {
            physicsSystem->Simulate(1.0f / 60.0f);
        }
    }

    void SimulationInterfaceTestFixture::TickApp(int numTicks)
    {
        AZ::ComponentApplication* app = nullptr;
        AZ::ComponentApplicationBus::BroadcastResult(app, &AZ::ComponentApplicationBus::Events::GetApplication);
        AZ_Assert(app, "Failed to get application");
        for (int i = 0; i < numTicks; i++)
        {
            app->Tick();
            // System tick drains AssetBus (so OnAssetReady notifications fire and assets transition
            // to Ready) and any SystemTickBus::QueueFunction() lambdas gems posted at Activate time.
            // Without it, SpawnableEntitiesManager requeues SpawnAllEntitiesCommand forever.
            app->TickSystem();
        }
    }

} // namespace UnitTest
