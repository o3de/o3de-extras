
/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <Common/SimulationInterfaceTestFixture.h>

#include <AzCore/Asset/AssetCommon.h>
#include <AzCore/Component/ComponentApplication.h>
#include <AzCore/Component/Entity.h>
#include <AzCore/UserSettings/UserSettingsComponent.h>
#include <AzCore/std/smart_ptr/unique_ptr.h>
#include <AzFramework/Components/TransformComponent.h>
#include <AzFramework/Spawnable/InMemorySpawnableAssetContainer.h>
#include <AzFramework/Spawnable/Spawnable.h>
#include <AzQtComponents/Utilities/QtPluginPaths.h>
#include <AzTest/GemTestEnvironment.h>
#include <AzToolsFramework/Entity/EditorEntityContextComponent.h>
#include <AzToolsFramework/ToolsComponents/TransformComponent.h>
#include <AzToolsFramework/UnitTest/AzToolsFrameworkTestHelpers.h>
#include <AzToolsFramework/UnitTest/ToolsTestApplication.h>
#include <Clients/SimulationEntitiesManager.h>
#include <Clients/SimulationManager.h>
#include <ROS2/Frame/ROS2FrameComponentBus.h>
#include <ROS2/ROS2TypeIds.h>
#include <SimulationInterfaces/SimulationEntityManagerRequestBus.h>

#include <QApplication>

namespace UnitTest
{
    //! Editor test environment: brings up a ToolsTestApplication (AzFramework + AzToolsFramework) so
    //! the editor SimulationInterfaces tests can exercise tools/editor APIs.
    class SimulationInterfaceTestEnvironment : public AZ::Test::GemTestEnvironment
    {
        void AddGemsAndComponents() override
        {
            constexpr AZStd::array<AZStd::string_view, 4> requiredGems = { "PhysX5", // required for PhysX Dynamic
                                                                           "LmbrCentral", // for shapes
                                                                           "ROS2", // For frame component
                                                                           "SimulationInterfaces" };
            AddActiveGems(requiredGems);
            AddDynamicModulePaths({ "PhysX5.Gem" });
            AddDynamicModulePaths({ "LmbrCentral" });
            AddDynamicModulePaths({ "ROS2" });
            AddComponentDescriptors(
                AZStd::initializer_list<AZ::ComponentDescriptor*>{ SimulationInterfaces::SimulationEntitiesManager::CreateDescriptor(),
                                                                   SimulationInterfaces::SimulationManager::CreateDescriptor() });
            AddRequiredComponents({ SimulationInterfaces::SimulationEntitiesManager::TYPEINFO_Uuid(),
                                    SimulationInterfaces::SimulationManager::TYPEINFO_Uuid() });
        }

        AZ::ComponentApplication* CreateApplicationInstance() override
        {
            // Using ToolsTestApplication to have AzFramework and AzToolsFramework components.
            return aznew UnitTest::ToolsTestApplication("SimulationInterfaceTestEnvironment");
        }

    protected:
        void PostSystemEntityActivate() override
        {
            AZ::UserSettingsComponentRequestBus::Broadcast(&AZ::UserSettingsComponentRequests::DisableSaveOnFinalize);
        }
    };

    //! Extends the editor test environment by assembling spawnable in memory, so SpawnEntity-based tests have a ready asset to spawn from.
    class SimulationInterfaceTestEnvironmentWithAssets : public SimulationInterfaceTestEnvironment
    {
    protected:
        void PostSystemEntityActivate() override;
        void PreDestroyApplication() override;

    private:
        //! Plus ".spawnable", this is the product path the tests' product_asset URI resolves to.
        static constexpr const char* TestSpawnableName = "sampleasset/testsimulationentity";

        static constexpr const char* TestSpawnableAssetId = "{6E0E1C39-2C6F-4C3E-9C31-1D0D6D9B5A77}:0";

        //! PreDestroyApplication needs to destroy the spawnable before GemTestEnvironment gets deleted
        AZStd::unique_ptr<AzFramework::InMemorySpawnableAssetContainer> m_spawnableAssets;
    };

    void SimulationInterfaceTestEnvironmentWithAssets::PostSystemEntityActivate()
    {
        SimulationInterfaceTestEnvironment::PostSystemEntityActivate();

        // Ownership passes to the asset registered below.
        auto* spawnable =
            aznew AzFramework::Spawnable(AZ::Data::AssetId::CreateString(TestSpawnableAssetId), AZ::Data::AssetData::AssetStatus::Ready);
        AzFramework::Spawnable::EntityList& entities = spawnable->GetEntities();

        auto root = AZStd::make_unique<AZ::Entity>("TestSimulationEntity");
        root->CreateComponent<AzFramework::TransformComponent>();

        auto body = AZStd::make_unique<AZ::Entity>("TestSimulationEntityBody");
        body->CreateComponent<AzFramework::TransformComponent>()->SetParent(root->GetId());
        body->CreateComponent(AZ::Uuid(PhysXRigidBodyComponentTypeId));
        body->CreateComponent(AZ::Uuid(PhysXShapeColliderComponentTypeId));
        body->CreateComponent(AZ::Uuid(SphereShapeComponentTypeId));
        body->CreateComponent(AZ::Uuid(ROS2::ROS2FrameComponentTypeId));

        entities.push_back(AZStd::move(root));
        entities.push_back(AZStd::move(body));

        AZ::Data::AssetInfo assetInfo;
        assetInfo.m_assetId = spawnable->GetId();
        assetInfo.m_assetType = azrtti_typeid<AzFramework::Spawnable>();
        assetInfo.m_relativePath = AZStd::string(TestSpawnableName) + AzFramework::Spawnable::DotFileExtension;

        AzFramework::InMemorySpawnableAssetContainer::AssetDataInfoContainer products;
        products.emplace_back(spawnable, assetInfo);

        m_spawnableAssets = AZStd::make_unique<AzFramework::InMemorySpawnableAssetContainer>();
        constexpr bool loadReferencedAssets = false;
        [[maybe_unused]] const auto result =
            m_spawnableAssets->CreateInMemorySpawnableAsset(products, loadReferencedAssets, TestSpawnableName);
        AZ_Assert(result.IsSuccess(), "Failed to register the test spawnable: %s", result.IsSuccess() ? "" : result.GetError().c_str());
    }

    void SimulationInterfaceTestEnvironmentWithAssets::PreDestroyApplication()
    {
        // The container's destructor does not unregister, so clear it while the catalog is still up.
        if (m_spawnableAssets)
        {
            m_spawnableAssets->ClearAllInMemorySpawnableAssets();
        }
        m_spawnableAssets.reset();
    }

    int getNumberOfEntities()
    {
        using namespace SimulationInterfaces;
        AZ::Outcome<EntityNameList, FailedResult> enitities;
        SimulationEntityManagerRequestBus::BroadcastResult(
            enitities, &SimulationEntityManagerRequestBus::Events::GetEntities, EntityFilters());
        AZ_Assert(enitities.IsSuccess(), "Failed to get entities");
        return enitities.GetValue().size();
    }

    TEST_F(SimulationInterfaceTestFixture, SpawnAppTest)
    {
        // This is an integration test that runs the test application with the SimulationInterfaces gem enabled.
        // The test environment registers the test spawnable, so entities can be spawned from it by URI.

        using namespace SimulationInterfaces;
        constexpr AZStd::string_view entityName = "MySuperDuperEntity";
        const AZ::Transform initialPose = AZ::Transform::CreateTranslation(AZ::Vector3(0.0f, 0.0f, 0.0f));
        constexpr AZStd::string_view uri = "product_asset:///sampleasset/testsimulationentity.spawnable";
        constexpr AZStd::string_view entityNamespace = "FooNamespace";
        AZStd::atomic_bool completed = false;
        SpawnCompletedCb completedCb = [&](const AZ::Outcome<AZStd::string, FailedResult>& result)
        {
            EXPECT_TRUE(result.IsSuccess());
            completed = true;
        };
        PreInsertionCb preinsertionCB = [](const AZ::Outcome<AzFramework::SpawnableEntityContainerView, FailedResult>& outcome)
        {
        };

        constexpr bool allowRename = false;
        SimulationEntityManagerRequestBus::Broadcast(
            &SimulationEntityManagerRequestBus::Events::SpawnEntity,
            entityName,
            uri,
            entityNamespace,
            initialPose,
            allowRename,
            preinsertionCB,
            completedCb);

        // entities are spawned asynchronously, so we need to tick the app to let the entity be spawned
        TickApp(100);
        EXPECT_TRUE(completed);

        // try to spawn entity with the same name, expect failure
        AZStd::atomic_bool completed2 = false;
        SpawnCompletedCb failedSpawnCompletedCb = [&](const AZ::Outcome<AZStd::string, FailedResult>& result)
        {
            EXPECT_FALSE(result.IsSuccess());
            completed2 = true;
        };
        SimulationEntityManagerRequestBus::Broadcast(
            &SimulationEntityManagerRequestBus::Events::SpawnEntity,
            entityName,
            uri,
            entityNamespace,
            initialPose,
            allowRename,
            preinsertionCB,
            failedSpawnCompletedCb);
        EXPECT_TRUE(completed2);

        // list simulation entities
        AZ::Outcome<EntityNameList, FailedResult> entitiesResult;
        SimulationEntityManagerRequestBus::BroadcastResult(
            entitiesResult, &SimulationEntityManagerRequestBus::Events::GetEntities, EntityFilters());
        ASSERT_TRUE(entitiesResult.IsSuccess());
        const auto& entities = entitiesResult.GetValue();
        EXPECT_EQ(entities.size(), 1);

        ASSERT_FALSE(entities.empty()) << "Simulated Entities Empty";
        const AZStd::string spawnedEntityName = entities.front();
        printf("Spawned entity name %s\n", spawnedEntityName.c_str());

        // get entity id by name
        AZ::Outcome<AZ::EntityId, FailedResult> entityIdResult;
        SimulationEntityManagerRequestBus::BroadcastResult(
            entityIdResult, &SimulationEntityManagerRequestBus::Events::GetEntityId, spawnedEntityName);

        ASSERT_TRUE(entityIdResult.IsSuccess()) << "Failed to get entity id";
        const AZ::EntityId entityId = entityIdResult.GetValue();
        // check namespace
        AZStd::string entityNamespaceOut;
        ROS2::ROS2FrameComponentBus::EventResult(entityNamespaceOut, entityId, &ROS2::ROS2FrameComponentBus::Events::GetNamespace);
        EXPECT_EQ(entityNamespaceOut, entityNamespace);

        // run physics simulation
        StepPhysics(100);

        // Get entity state,
        AZ::Outcome<MultipleEntitiesStates, FailedResult> entityStatesResult;
        SimulationEntityManagerRequestBus::BroadcastResult(
            entityStatesResult, &SimulationEntityManagerRequestBus::Events::GetEntitiesStates, EntityFilters());
        ASSERT_TRUE(entityStatesResult.IsSuccess());

        const auto& entityStates = entityStatesResult.GetValue();
        auto entityState = entityStates.find(spawnedEntityName);
        ASSERT_NE(entityState, entityStates.end());
        EXPECT_EQ(entityState->first, spawnedEntityName);

        // check if the entity moved
        EXPECT_GE(entityState->second.m_pose.GetTranslation().GetDistance(initialPose.GetTranslation()), 1.0f);

        // set new entity state - move the entity to X=1000 meters
        const AZ::Vector3 newPosition = AZ::Vector3(1000.0f, 0.0f, 0.0f);
        const EntityState newState = { AZ::Transform::CreateTranslation(newPosition),
                                       AZ::Vector3::CreateZero(),
                                       AZ::Vector3::CreateZero() };
        SimulationEntityManagerRequestBus::Broadcast(
            &SimulationEntityManagerRequestBus::Events::SetEntityState, spawnedEntityName, newState);

        StepPhysics();

        // Check if entity was teleported by setting the new state, we use a filter to check if the entity is at the new position
        EntityFilters filter;
        filter.m_boundsShape = AZStd::make_shared<Physics::SphereShapeConfiguration>(2.0f);
        filter.m_boundsPose = AZ::Transform::CreateTranslation(AZ::Vector3(1000.0f, 0.0f, 0.0f));
        AZ::Outcome<EntityNameList, FailedResult> entitiesFiltered;
        SimulationEntityManagerRequestBus::BroadcastResult(
            entitiesFiltered, &SimulationEntityManagerRequestBus::Events::GetEntities, filter);
        ASSERT_TRUE(entitiesFiltered.IsSuccess());
        EXPECT_EQ(entitiesFiltered.GetValue().size(), 1);

        // delete entity using its name
        DeletionCompletedCb deletionCompletedCb = [](const AZ::Outcome<void, FailedResult>& result)
        {
            EXPECT_TRUE(result.IsSuccess());
        };
        SimulationEntityManagerRequestBus::Broadcast(
            &SimulationEntityManagerRequestBus::Events::DeleteEntity, entityName, deletionCompletedCb);
        TickApp(100);

        // check if the entity was deleted
        EXPECT_EQ(getNumberOfEntities(), 0);

        // spawn 3 entities of entities and despawn all of them
        SpawnCompletedCb cb = [&](const AZ::Outcome<AZStd::string, FailedResult>& result)
        {
        };
        SimulationEntityManagerRequestBus::Broadcast(
            &SimulationEntityManagerRequestBus::Events::SpawnEntity,
            "entity1",
            uri,
            entityNamespace,
            initialPose,
            false,
            preinsertionCB,
            cb);
        SimulationEntityManagerRequestBus::Broadcast(
            &SimulationEntityManagerRequestBus::Events::SpawnEntity,
            "entity2",
            uri,
            entityNamespace,
            initialPose,
            false,
            preinsertionCB,
            cb);
        SimulationEntityManagerRequestBus::Broadcast(
            &SimulationEntityManagerRequestBus::Events::SpawnEntity,
            "entity3",
            uri,
            entityNamespace,
            initialPose,
            false,
            preinsertionCB,
            cb);
        TickApp(100);
        EXPECT_EQ(getNumberOfEntities(), 3);

        // delete all entities
        bool deletionWasCompleted = false;
        DeletionCompletedCb deleteAllCompletion = [&deletionWasCompleted](const AZ::Outcome<void, FailedResult>& result)
        {
            deletionWasCompleted = true;
            EXPECT_TRUE(result.IsSuccess());
        };
        SimulationEntityManagerRequestBus::Broadcast(&SimulationEntityManagerRequestBus::Events::DeleteAllEntities, deleteAllCompletion);
        TickApp(100);
        EXPECT_TRUE(deletionWasCompleted);
        EXPECT_EQ(getNumberOfEntities(), 0);
    }

} // namespace UnitTest

// required to support running integration tests with Qt and PhysX
AZTEST_EXPORT int AZ_UNIT_TEST_HOOK_NAME(int argc, char** argv)
{
    ::testing::InitGoogleMock(&argc, argv);
    AzQtComponents::PrepareQtPaths();
    QApplication app(argc, argv);
    AZ::Test::printUnusedParametersWarning(argc, argv);
    AZ::Test::addTestEnvironments({ new UnitTest::SimulationInterfaceTestEnvironmentWithAssets() });
    int result = RUN_ALL_TESTS();
    return result;
}

IMPLEMENT_TEST_EXECUTABLE_MAIN();
