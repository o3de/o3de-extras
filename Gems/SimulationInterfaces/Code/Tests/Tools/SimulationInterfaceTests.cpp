
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

    TEST_F(SimulationInterfaceTestFixture, EmptyScene)
    {
        using namespace SimulationInterfaces;
        AZ::Outcome<EntityNameList, FailedResult> entities;
        SimulationEntityManagerRequestBus::BroadcastResult(
            entities, &SimulationEntityManagerRequestBus::Events::GetEntities, EntityFilters());
        ASSERT_TRUE(entities.IsSuccess());
        EXPECT_EQ(entities.GetValue().size(), 0);
    }

    TEST_F(SimulationInterfaceTestFixture, AddSimulatedEntityThenRemove)
    {
        using namespace SimulationInterfaces;
        const AZ::EntityId entityId1 = CreateEntityWithStaticBodyComponent("Foo", AZ::Transform::CreateIdentity());
        const AZ::EntityId entityId2 = CreateEntityWithStaticBodyComponent("Bar", AZ::Transform::CreateIdentity());

        AZ::Outcome<EntityNameList, FailedResult> entities;
        SimulationEntityManagerRequestBus::BroadcastResult(
            entities, &SimulationEntityManagerRequestBus::Events::GetEntities, EntityFilters());
        ASSERT_TRUE(entities.IsSuccess());
        ASSERT_EQ(entities.GetValue().size(), 2);
        DeleteEntity(entityId1);

        AZ::Outcome<EntityNameList, FailedResult> entities2;
        SimulationEntityManagerRequestBus::BroadcastResult(
            entities2, &SimulationEntityManagerRequestBus::Events::GetEntities, EntityFilters());
        ASSERT_TRUE(entities2.IsSuccess());
        EXPECT_EQ(entities2.GetValue().size(), 1);

        DeleteEntity(entityId2);
        AZ::Outcome<EntityNameList, FailedResult> entities3;
        SimulationEntityManagerRequestBus::BroadcastResult(
            entities3, &SimulationEntityManagerRequestBus::Events::GetEntities, EntityFilters());
        ASSERT_TRUE(entities3.IsSuccess());
        EXPECT_EQ(entities3.GetValue().size(), 0);
    }

    TEST_F(SimulationInterfaceTestFixture, AddEntitiesWithDupName)
    {
        using namespace SimulationInterfaces;

        const AZ::EntityId entityId1 = CreateEntityWithStaticBodyComponent("Bar1", AZ::Transform::CreateIdentity());
        const AZ::EntityId entityId2 = CreateEntityWithStaticBodyComponent("Bar1", AZ::Transform::CreateIdentity());

        AZ::Outcome<EntityNameList, FailedResult> entities;

        SimulationEntityManagerRequestBus::BroadcastResult(
            entities, &SimulationEntityManagerRequestBus::Events::GetEntities, EntityFilters());
        ASSERT_TRUE(entities.IsSuccess());
        EXPECT_EQ(entities.GetValue().size(), 2);
        EXPECT_NE(entities.GetValue()[0], entities.GetValue()[1]);
        DeleteEntity(entityId1);
        DeleteEntity(entityId2);
    }

    TEST_F(SimulationInterfaceTestFixture, TestShapeFilter)
    {
        // This test is disabled since due to some issue outside to this gem, the rigid body is created without the collider shape
        // and the filter is not applied. This test will be enabled once the issue is resolved.
        GTEST_SKIP() << "Need to fix the issue with the collider shape creation.";
        using namespace SimulationInterfaces;
        const AZ::EntityId entityId1 =
            CreateEntityWithStaticBodyComponent("Inside", AZ::Transform::CreateTranslation(AZ::Vector3(0.0f, 0.0f, 0.0f)));
        const AZ::EntityId entityId2 =
            CreateEntityWithStaticBodyComponent("Outside", AZ::Transform::CreateTranslation(AZ::Vector3(10.0f, 0.0f, 0.0f)));

        EntityFilters filter;
        filter.m_bounds_shape = AZStd::make_shared<Physics::SphereShapeConfiguration>(2.0f);

        AZ::Outcome<EntityNameList, FailedResult> entities;
        SimulationEntityManagerRequestBus::BroadcastResult(entities, &SimulationEntityManagerRequestBus::Events::GetEntities, filter);
        auto* physicsSystem = AZ::Interface<AzPhysics::SystemInterface>::Get();
        physicsSystem->Simulate(1.0f / 60.0f);

        ASSERT_TRUE(entities.IsSuccess());
        ASSERT_EQ(entities.GetValue().size(), 1);
        EXPECT_EQ(entities.GetValue().front(), "Inside");

        DeleteEntity(entityId1);
        DeleteEntity(entityId2);
    }

    TEST_F(SimulationInterfaceTestFixture, TestRegexFilter)
    {
        using namespace SimulationInterfaces;
        const AZ::EntityId entityId1 =
            CreateEntityWithStaticBodyComponent("WillMatch", AZ::Transform::CreateTranslation(AZ::Vector3(0.0f, 0.0f, 0.0f)));
        const AZ::EntityId entityId2 =
            CreateEntityWithStaticBodyComponent("WontMatch", AZ::Transform::CreateTranslation(AZ::Vector3(10.0f, 0.0f, 0.0f)));

        EntityFilters filter;
        filter.m_filter = "Will.*";

        AZ::Outcome<EntityNameList, FailedResult> entities;
        SimulationEntityManagerRequestBus::BroadcastResult(entities, &SimulationEntityManagerRequestBus::Events::GetEntities, filter);

        ASSERT_TRUE(entities.IsSuccess());
        ASSERT_EQ(entities.GetValue().size(), 1);
        EXPECT_EQ(entities.GetValue().front(), "WillMatch");

        DeleteEntity(entityId1);
        DeleteEntity(entityId2);
    }

    TEST_F(SimulationInterfaceTestFixture, TestRegexFilterInvalid)
    {
        // Invalid regex should not match any entity
        using namespace SimulationInterfaces;
        const AZ::EntityId entityId1 =
            CreateEntityWithStaticBodyComponent("WillMatch", AZ::Transform::CreateTranslation(AZ::Vector3(0.0f, 0.0f, 0.0f)));
        const AZ::EntityId entityId2 =
            CreateEntityWithStaticBodyComponent("WontMatch", AZ::Transform::CreateTranslation(AZ::Vector3(10.0f, 0.0f, 0.0f)));

        EntityFilters filter;
        filter.m_filter = "[a-z";

        AZ::Outcome<EntityNameList, FailedResult> entities;
        SimulationEntityManagerRequestBus::BroadcastResult(entities, &SimulationEntityManagerRequestBus::Events::GetEntities, filter);

        EXPECT_FALSE(entities.IsSuccess());

        DeleteEntity(entityId1);
        DeleteEntity(entityId2);
    }

    TEST_F(SimulationInterfaceTestFixture, SmokeTestGetEntityState)
    {
        // Invalid regex should not match any entity
        using namespace SimulationInterfaces;
        const AZStd::string entityName = "DroppedBall";
        const AZ::EntityId entityId1 =
            CreateEntityWithStaticBodyComponent(entityName, AZ::Transform::CreateTranslation(AZ::Vector3(2.0f, 0.0f, 10.0f)));

        EntityFilters filter;
        AZ::Outcome<EntityState, FailedResult> stateBeforeResult;
        SimulationEntityManagerRequestBus::BroadcastResult(
            stateBeforeResult, &SimulationEntityManagerRequestBus::Events::GetEntityState, entityName);
        EXPECT_TRUE(stateBeforeResult.IsSuccess());
        const auto& stateBefore = stateBeforeResult.GetValue();
        EXPECT_EQ(stateBefore.m_pose.GetTranslation(), AZ::Vector3(2.0f, 0.0f, 10.0f));
        for (int i = 0; i < 10; i++)
        {
            auto* physicsSystem = AZ::Interface<AzPhysics::SystemInterface>::Get();
            physicsSystem->Simulate(1.0f / 60.0f);
        }
        AZ::Outcome<EntityState, FailedResult> stateAfterResult;
        SimulationEntityManagerRequestBus::BroadcastResult(
            stateAfterResult, &SimulationEntityManagerRequestBus::Events::GetEntityState, entityName);
        EXPECT_TRUE(stateAfterResult.IsSuccess());
        const auto& stateAfter = stateAfterResult.GetValue();
        AZ::Vector3 deltaPos = stateAfter.m_pose.GetTranslation() - stateBefore.m_pose.GetTranslation();

        // check if the entity moved
        EXPECT_GT(deltaPos.GetLength(), 0.0f);

        // check if entity has velocity
        EXPECT_GT(stateAfter.m_twist_linear.GetLength(), 0.0f);

        DeleteEntity(entityId1);
    }

} // namespace UnitTest

// required to support running integration tests with Qt and PhysX
AZTEST_EXPORT int AZ_UNIT_TEST_HOOK_NAME(int argc, char** argv)
{
    ::testing::InitGoogleMock(&argc, argv);
    AzQtComponents::PrepareQtPaths();
    QApplication app(argc, argv);
    AZ::Test::printUnusedParametersWarning(argc, argv);
    AZ::Test::addTestEnvironments({ new UnitTest::SimulationInterfaceTestEnvironment() });
    int result = RUN_ALL_TESTS();
    return result;
}

IMPLEMENT_TEST_EXECUTABLE_MAIN();
