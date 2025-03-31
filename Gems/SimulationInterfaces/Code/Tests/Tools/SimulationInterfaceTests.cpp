
/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "TestFixture.h"
#include <SimulationInterfaces/SimulationInterfacesBus.h>
namespace UnitTest
{

    TEST_F(SimulationInterfaceTestFixture, EmptyScene)
    {
        using namespace SimulationInterfaces;
        AZStd::vector<AZStd::string> entities;
        SimulationInterfacesRequestBus::BroadcastResult(entities, &SimulationInterfacesRequestBus::Events::GetEntities, EntityFilter());
        EXPECT_EQ(entities.size(), 0);
    }

    TEST_F(SimulationInterfaceTestFixture, AddSimulatedEntityThenRemove)
    {
        using namespace SimulationInterfaces;
        const AZ::EntityId entityId1 = CreateEntityWithStaticBodyComponent("Foo", AZ::Transform::CreateIdentity());
        const AZ::EntityId entityId2 = CreateEntityWithStaticBodyComponent("Bar", AZ::Transform::CreateIdentity());

        AZStd::vector<AZStd::string> entities;
        SimulationInterfacesRequestBus::BroadcastResult(entities, &SimulationInterfacesRequestBus::Events::GetEntities, EntityFilter());
        AZ_Assert(entities.size() == 2, "Number of simulation entities: %d", entities.size());
        DeleteEntity(entityId1);

        AZStd::vector<AZStd::string> entities2;
        SimulationInterfacesRequestBus::BroadcastResult(entities2, &SimulationInterfacesRequestBus::Events::GetEntities, EntityFilter());
        EXPECT_EQ(entities2.size(), 1);

        DeleteEntity(entityId2);
        AZStd::vector<AZStd::string> entities3;
        SimulationInterfacesRequestBus::BroadcastResult(entities3, &SimulationInterfacesRequestBus::Events::GetEntities, EntityFilter());
        EXPECT_EQ(entities3.size(), 0);
    }

    TEST_F(SimulationInterfaceTestFixture, AddEntitiesWithDupName)
    {
        using namespace SimulationInterfaces;

        const AZ::EntityId entityId1 = CreateEntityWithStaticBodyComponent("Bar1", AZ::Transform::CreateIdentity());
        const AZ::EntityId entityId2 = CreateEntityWithStaticBodyComponent("Bar1", AZ::Transform::CreateIdentity());
        AZStd::vector<AZStd::string> entities;

        SimulationInterfacesRequestBus::BroadcastResult(entities, &SimulationInterfacesRequestBus::Events::GetEntities, EntityFilter());
        EXPECT_EQ(entities.size(), 2);
        EXPECT_NE(entities[0], entities[1]);
        DeleteEntity(entityId1);
        DeleteEntity(entityId2);
    }

    TEST_F(SimulationInterfaceTestFixture, TestShapeFilter)
    {
        // This test is disabled since due to some issue outside to this gem, the rigid body is created without the collider shape
        // and the filter is not applied. This test will be enabled once the issue is resolved.
        return;
        using namespace SimulationInterfaces;
        const AZ::EntityId entityId1 =
            CreateEntityWithStaticBodyComponent("Inside", AZ::Transform::CreateTranslation(AZ::Vector3(0.0f, 0.0f, 0.0f)));
        const AZ::EntityId entityId2 =
            CreateEntityWithStaticBodyComponent("Outside", AZ::Transform::CreateTranslation(AZ::Vector3(10.0f, 0.0f, 0.0f)));

        EntityFilter filter;
        filter.m_bounds_shape = AZStd::make_shared<Physics::SphereShapeConfiguration>(2.0f);

        AZStd::vector<AZStd::string> entities;
        SimulationInterfacesRequestBus::BroadcastResult(entities, &SimulationInterfacesRequestBus::Events::GetEntities, filter);
        auto* physicsSystem = AZ::Interface<AzPhysics::SystemInterface>::Get();
        physicsSystem->Simulate(1.0f / 60.0f);

        EXPECT_EQ(entities.size(), 1);
        if (entities.size() > 0)
        {
            EXPECT_EQ(entities.front(), "Inside");
        }
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

        EntityFilter filter;
        filter.m_filter = "Will.*";

        AZStd::vector<AZStd::string> entities;
        SimulationInterfacesRequestBus::BroadcastResult(entities, &SimulationInterfacesRequestBus::Events::GetEntities, filter);

        EXPECT_EQ(entities.size(), 1);
        if (entities.size() > 0)
        {
            EXPECT_EQ(entities.front(), "WillMatch");
        }
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

        EntityFilter filter;
        filter.m_filter = "[a-z";

        AZStd::vector<AZStd::string> entities;
        SimulationInterfacesRequestBus::BroadcastResult(entities, &SimulationInterfacesRequestBus::Events::GetEntities, filter);

        EXPECT_EQ(entities.size(), 0);
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

        EntityFilter filter;
        EntityState stateBefore;
        SimulationInterfacesRequestBus::BroadcastResult(stateBefore, &SimulationInterfacesRequestBus::Events::GetEntityState, entityName);
        EXPECT_EQ(stateBefore.m_pose.GetTranslation(), AZ::Vector3(2.0f, 0.0f, 10.0f));
        for (int i = 0; i < 10; i++)
        {
            auto* physicsSystem = AZ::Interface<AzPhysics::SystemInterface>::Get();
            physicsSystem->Simulate(1.0f / 60.0f);
        }
        EntityState stateAfter;
        SimulationInterfacesRequestBus::BroadcastResult(stateAfter, &SimulationInterfacesRequestBus::Events::GetEntityState, entityName);
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
