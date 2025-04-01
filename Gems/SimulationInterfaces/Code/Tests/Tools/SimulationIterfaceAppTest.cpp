
/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "TestFixture.h"
#include <SimulationInterfaces/SimulationEntityManagerRequestBus.h>

namespace UnitTest
{
    class SimulationInterfaceTestEnvironmentWithAssets : public SimulationInterfaceTestEnvironment
    {
    protected:
        void PostSystemEntityActivate();
    };

    void SimulationInterfaceTestEnvironmentWithAssets::PostSystemEntityActivate()
    {
        // Prepare the asset catalog and ensure that our test asset (testsimulationentity.spawnable) is loaded and
        // ready to be used in test scenarios.
        AZ::UserSettingsComponentRequestBus::Broadcast(&AZ::UserSettingsComponentRequests::DisableSaveOnFinalize);

        AZ::ComponentApplication* app = nullptr;
        AZ::ComponentApplicationBus::BroadcastResult(app, &AZ::ComponentApplicationBus::Events::GetApplication);
        AZ_Assert(app, "Failed to get application");
        auto products = AZ::Utils::GetProjectProductPathForPlatform().c_str();
        AZ::IO::Path assetCatalogPath = AZ::IO::Path(products) / "assetcatalog.xml";
        bool catalogExists = AZ::IO::FileIOBase::GetInstance()->Exists(assetCatalogPath.c_str());
        AZ_Assert(catalogExists, "Asset Catalog in %s does not exist", assetCatalogPath.c_str());

        AZ::Data::AssetCatalogRequestBus::Broadcast(&AZ::Data::AssetCatalogRequestBus::Events::LoadCatalog, assetCatalogPath.c_str());

        const AZ::IO::Path TestSpawnable = "sampleasset/testsimulationentity.spawnable";
        const AZ::IO::Path TestSpawnableGlobalPath = AZ::IO::Path(products) / TestSpawnable;
        bool spawnableExists = AZ::IO::FileIOBase::GetInstance()->Exists(assetCatalogPath.c_str());
        AZ_Assert(spawnableExists, "%s does not exist", TestSpawnableGlobalPath.c_str());

        AZ::Data::AssetId assetId;
        AZ::Data::AssetCatalogRequestBus::BroadcastResult(
            assetId,
            &AZ::Data::AssetCatalogRequestBus::Events::GetAssetIdByPath,
            TestSpawnable.c_str(),
            AZ::Data::s_invalidAssetType,
            false);
        AZ_Assert(assetId.IsValid(), "Failed to get asset id for %s", TestSpawnable.c_str());
    }


    TEST_F(SimulationInterfaceTestFixture, SpawnAppTest)
    {
        // This is an integration test that runs the test application with the SimulationInterfaces gem enabled.
        // It has prepared asset catalog, and we are able to spawn entities with the test asset.

        using namespace SimulationInterfaces;
        constexpr AZStd::string_view entityName = "MySuperDuperEntity";
        const AZ::Transform initialPose = AZ::Transform::CreateTranslation(AZ::Vector3(0.0f, 0.0f, 0.0f));
        constexpr AZStd::string_view uri = "product_asset:///sampleasset/testsimulationentity.spawnable";
        constexpr AZStd::string_view entityNamespace = "";
        AZStd::atomic_bool completed = false;
        SimulationEntityManagerRequests::SpawnCompletedCb completedCb = [&](const AZ::Outcome<AZStd::string, AZStd::string>& result)
        {
            EXPECT_TRUE(result.IsSuccess());
            completed = true;
        };

        SimulationEntityManagerRequestBus::Broadcast(
            &SimulationEntityManagerRequestBus::Events::SpawnEntity, entityName, uri, entityNamespace, initialPose, completedCb);

        // entities are spawned asynchronously, so we need to tick the app to let the entity be spawned
        TickApp(100);
        EXPECT_TRUE(completed);

        // list simulation entities
        AZStd::vector<AZStd::string> entities;
        SimulationEntityManagerRequestBus::BroadcastResult(entities, &SimulationEntityManagerRequestBus::Events::GetEntities, EntityFilters());
        EXPECT_EQ(entities.size(), 1);

        AZ_Assert(!entities.empty(), "Simulated Entities Empty");
        const AZStd::string spawnedEntityName = entities.front();
        printf("Spawned entity name %s\n", spawnedEntityName.c_str());

        // run physics simulation
        StepPhysics(100);

        // Get entity state,
        AZStd::unordered_map<AZStd::string, EntityState> entityStates;
        SimulationEntityManagerRequestBus::BroadcastResult(
            entityStates, &SimulationEntityManagerRequestBus::Events::GetEntitiesStates, EntityFilters());
        auto entityState = entityStates.find(spawnedEntityName);
        EXPECT_NE(entityState, entityStates.end());
        EXPECT_EQ(entityState->first, spawnedEntityName);

        // check if the entity moved
        EXPECT_GE(entityState->second.m_pose.GetTranslation().GetDistance(initialPose.GetTranslation()), 1.0f);

        // set new entity state - move the entity to X=1000 meters
        const AZ::Vector3 newPosition = AZ::Vector3(1000.0f, 0.0f, 0.0f);
        const EntityState newState = { AZ::Transform::CreateTranslation(newPosition), AZ::Vector3::CreateZero(), AZ::Vector3::CreateZero() };
        SimulationEntityManagerRequestBus::Broadcast(&SimulationEntityManagerRequestBus::Events::SetEntityState, spawnedEntityName, newState);

        StepPhysics();

        // Check if entity was teleported by setting the new state, we use a filter to check if the entity is at the new position
        EntityFilters filter;
        filter.m_bounds_shape = AZStd::make_shared<Physics::SphereShapeConfiguration>(2.0f);
        filter.m_bounds_pose = AZ::Transform::CreateTranslation(AZ::Vector3(1000.0f, 0.0f, 0.0f));
        AZStd::vector<AZStd::string> entitiesFiltered;
        SimulationEntityManagerRequestBus::BroadcastResult(entitiesFiltered, &SimulationEntityManagerRequestBus::Events::GetEntities, filter);
        EXPECT_EQ(entitiesFiltered.size(), 1);

        // delete entity using its name
        SimulationEntityManagerRequestBus::Broadcast(&SimulationEntityManagerRequestBus::Events::DeleteEntity, entityName);
        TickApp(100);

        // list simulation entities after deletion, expect no simulation entities
        AZStd::vector<AZStd::string> entities2;
        SimulationEntityManagerRequestBus::BroadcastResult(entities2, &SimulationEntityManagerRequestBus::Events::GetEntities, EntityFilters());
        EXPECT_EQ(entities2.size(), 0);
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
