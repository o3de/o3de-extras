
/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <AzCore/Asset/AssetManagerComponent.h>
#include <AzCore/Component/ComponentApplication.h>
#include <AzCore/Component/ComponentApplicationBus.h>
#include <AzCore/Component/Entity.h>
#include <AzCore/Component/EntityId.h>
#include <AzCore/RTTI/RTTIMacros.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/Slice/SliceAssetHandler.h>
#include <AzCore/UserSettings/UserSettingsComponent.h>
#include <AzCore/std/containers/array.h>
#include <AzCore/std/string/string_view.h>
#include <AzQtComponents/Utilities/QtPluginPaths.h>
#include <AzTest/GemTestEnvironment.h>
#include <AzToolsFramework/Entity/EditorEntityContextComponent.h>
#include <AzToolsFramework/ToolsComponents/TransformComponent.h>
#include <AzToolsFramework/UnitTest/AzToolsFrameworkTestHelpers.h>
#include <AzToolsFramework/UnitTest/ToolsTestApplication.h>
#include <Clients/ROS2SystemComponent.h>
#include <Frame/ROS2FrameSystemComponent.h>
#include <ROS2/Frame/ROS2FrameComponent.h>
#include <ROS2/Frame/ROS2FrameTrackingInterface.h>
#include <ROS2/ROS2Bus.h>

#include <QApplication>
#include <gtest/gtest.h>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

namespace UnitTest
{
    class ROS2FrameComponentTestEnvironment : public AZ::Test::GemTestEnvironment
    {
        // AZ::Test::GemTestEnvironment overrides ...
        void AddGemsAndComponents() override;
        AZ::ComponentApplication* CreateApplicationInstance() override;
        void PostSystemEntityActivate() override;

    public:
        ROS2FrameComponentTestEnvironment() = default;
        ~ROS2FrameComponentTestEnvironment() override = default;
    };

    void ROS2FrameComponentTestEnvironment::AddGemsAndComponents()
    {
        AddActiveGems(AZStd::to_array<AZStd::string_view>({ "ROS2" }));
        AddDynamicModulePaths({});
        AddComponentDescriptors(AZStd::initializer_list<AZ::ComponentDescriptor*>{ ROS2::ROS2FrameComponent::CreateDescriptor(),
                                                                                   ROS2::ROS2SystemComponent::CreateDescriptor(),
                                                                                   ROS2::ROS2FrameSystemComponent::CreateDescriptor() });
        AddRequiredComponents(AZStd::to_array<AZ::TypeId const>(
            { ROS2::ROS2SystemComponent::TYPEINFO_Uuid(), ROS2::ROS2FrameSystemComponent::TYPEINFO_Uuid() }));
    }

    AZ::ComponentApplication* ROS2FrameComponentTestEnvironment::CreateApplicationInstance()
    {
        // Using ToolsTestApplication to have AzFramework and AzToolsFramework components.
        return aznew UnitTest::ToolsTestApplication("ROS2FrameComponent");
    }

    void ROS2FrameComponentTestEnvironment::PostSystemEntityActivate()
    {
        AZ::UserSettingsComponentRequestBus::Broadcast(&AZ::UserSettingsComponentRequests::DisableSaveOnFinalize);
    }

    class ROS2FrameComponentFixture : public ::testing::Test
    {
    };

    TEST_F(ROS2FrameComponentFixture, SingleFrameDefault)
    {
        ROS2::ROS2FrameConfiguration config;

        AZ::Entity entity;
        const std::string entityName = entity.GetName().c_str();
        entity.CreateComponent<AzFramework::TransformComponent>();
        auto frame = entity.CreateComponent<ROS2::ROS2FrameComponent>(config);

        entity.Init();
        entity.Activate();

        const std::string jointName(frame->GetNamespacedJointName().GetCStr());
        const std::string frameId(frame->GetNamespacedFrameID().c_str());

        EXPECT_EQ(entity.GetState(), AZ::Entity::State::Active);
        EXPECT_STRCASEEQ(jointName.c_str(), ("o3de_" + entityName + "/").c_str());
        EXPECT_STRCASEEQ(frameId.c_str(), ("o3de_" + entityName + "/sensor_frame").c_str());
    }

    TEST_F(ROS2FrameComponentFixture, ThreeFramesDefault)
    {
        ROS2::ROS2FrameConfiguration config;

        std::vector<AZStd::unique_ptr<AZ::Entity>> entities;
        std::vector<const ROS2::ROS2FrameComponent*> frames;

        constexpr int numOfEntities = 3;

        for (int i = 0; i < numOfEntities; i++)
        {
            entities.push_back(AZStd::make_unique<AZ::Entity>());
        }

        for (int i = 0; i < numOfEntities; i++)
        {
            entities[i]->CreateComponent<AzFramework::TransformComponent>();
            entities[i]->SetName(("entity" + std::to_string(i)).c_str());
            entities[i]->Init();
            entities[i]->Activate();
            if (i != 0)
            {
                AZ::TransformBus::Event(entities[i]->GetId(), &AZ::TransformBus::Events::SetParent, entities[i - 1]->GetId());
            }
            entities[i]->Deactivate();

            auto frame = entities[i]->CreateComponent<ROS2::ROS2FrameComponent>(config);
            frames.push_back(frame);

            entities[i]->Activate();
        }

        for (int i = 0; i < numOfEntities; i++)
        {
            const std::string jointName(frames[i]->GetNamespacedJointName().GetCStr());
            const std::string frameId(frames[i]->GetNamespacedFrameID().c_str());

            EXPECT_EQ(entities[i]->GetState(), AZ::Entity::State::Active);
            EXPECT_STRCASEEQ(jointName.c_str(), (entities[0]->GetName() + "/").c_str());
            EXPECT_STRCASEEQ(frameId.c_str(), (entities[0]->GetName() + "/sensor_frame").c_str());
        }
    }

    TEST_F(ROS2FrameComponentFixture, UpdateNamespace)
    {
        ROS2::ROS2FrameConfiguration config;

        AZ::Entity entity;
        const std::string entityName = entity.GetName().c_str();
        entity.CreateComponent<AzFramework::TransformComponent>();
        auto frame = entity.CreateComponent<ROS2::ROS2FrameComponent>(config);

        entity.Init();
        entity.Activate();

        auto rosifiedName = "o3de_" + entityName;
        ASSERT_STREQ(frame->GetNamespace().c_str(), rosifiedName.c_str());

        // Note that namespace parameter is only applied using Custom strategy
        frame->UpdateNamespaceConfiguration("MyCustomNamespace", ROS2::NamespaceConfiguration::NamespaceStrategy::Custom);
        EXPECT_STREQ(frame->GetNamespace().c_str(), "MyCustomNamespace");
        // Empty strategy clears the namespace
        frame->UpdateNamespaceConfiguration("MyCustomNamespace", ROS2::NamespaceConfiguration::NamespaceStrategy::Empty);
        EXPECT_STREQ(frame->GetNamespace().c_str(), "");
        // From Entity Name strategy uses rosified version of Entity Name
        frame->UpdateNamespaceConfiguration("MyCustomNamespace", ROS2::NamespaceConfiguration::NamespaceStrategy::FromEntityName);
        EXPECT_STREQ(frame->GetNamespace().c_str(), rosifiedName.c_str());
        // Default strategy is like From Entity Name strategy if the entity is on top of hierarchy ...
        frame->UpdateNamespaceConfiguration("MyCustomNamespace", ROS2::NamespaceConfiguration::NamespaceStrategy::Default);
        EXPECT_STREQ(frame->GetNamespace().c_str(), rosifiedName.c_str());

        AZ::Entity newRootEntity;
        AZStd::string newRootName = "new_root";
        newRootEntity.SetName(newRootName);
        newRootEntity.CreateComponent<AzFramework::TransformComponent>();
        newRootEntity.CreateComponent<ROS2::ROS2FrameComponent>(config);
        newRootEntity.Init();
        newRootEntity.Activate();
        AZ::TransformBus::Event(entity.GetId(), &AZ::TransformBus::Events::SetParent, newRootEntity.GetId());

        // If there is a parent Default strategy concatenates parent's namespace with rosified name
        frame->UpdateNamespaceConfiguration("MyCustomNamespace", ROS2::NamespaceConfiguration::NamespaceStrategy::Default);
        EXPECT_STREQ(frame->GetNamespace().c_str(), AZStd::string::format("%s/%s", newRootName.c_str(), rosifiedName.c_str()).c_str());
    }

    TEST_F(ROS2FrameComponentFixture, FrameTrackingInterface_EmptyRegistry)
    {
        // Check if the tracking interface is available
        auto* trackingInterface = ROS2::ROS2FrameTrackingInterface::Get();
        ASSERT_NE(trackingInterface, nullptr)
            << "ROS2FrameTrackingInterface not available - ROS2FrameSystemComponent may not be properly initialized";

        // Test tracking interface with no frames registered
        EXPECT_EQ(trackingInterface->GetRegisteredFrameCount(), 0);

        const AZStd::unordered_set<AZ::EntityId>& registeredFrames = trackingInterface->GetRegisteredFrames();
        EXPECT_TRUE(registeredFrames.empty());

        auto allFrameIds = trackingInterface->GetAllNamespacedFrameIds();
        EXPECT_TRUE(allFrameIds.empty());
    }

    TEST_F(ROS2FrameComponentFixture, FrameTrackingInterface_SingleFrame)
    {
        auto* trackingInterface = ROS2::ROS2FrameTrackingInterface::Get();
        ASSERT_NE(trackingInterface, nullptr) << "ROS2FrameTrackingInterface not available";

        ROS2::ROS2FrameConfiguration config;

        AZ::Entity entity;
        entity.SetName("test_entity");
        entity.CreateComponent<AzFramework::TransformComponent>();
        auto frame = entity.CreateComponent<ROS2::ROS2FrameComponent>(config);

        entity.Init();
        entity.Activate();

        // Test frame count
        EXPECT_EQ(trackingInterface->GetRegisteredFrameCount(), 1);

        // Test frame registration check
        EXPECT_TRUE(trackingInterface->IsFrameRegistered(entity.GetId()));

        // Test getting registered frames
        const AZStd::unordered_set<AZ::EntityId>& registeredFrames = trackingInterface->GetRegisteredFrames();
        EXPECT_EQ(registeredFrames.size(), 1);
        EXPECT_TRUE(registeredFrames.contains(entity.GetId()));

        // Test getting namespaced frame ID
        AZStd::string namespacedFrameId = trackingInterface->GetNamespacedFrameId(entity.GetId());
        EXPECT_FALSE(namespacedFrameId.empty());
        EXPECT_STREQ(namespacedFrameId.c_str(), frame->GetNamespacedFrameID().c_str());

        // Test getting entity by namespaced frame ID
        AZ::EntityId foundEntityId = trackingInterface->GetFrameEntityByNamespacedId(namespacedFrameId);
        EXPECT_EQ(foundEntityId, entity.GetId());

        // Test getting all namespaced frame IDs
        auto allFrameIds = trackingInterface->GetAllNamespacedFrameIds();
        EXPECT_EQ(allFrameIds.size(), 1);
        EXPECT_TRUE(allFrameIds.contains(namespacedFrameId));
    }

    TEST_F(ROS2FrameComponentFixture, FrameTrackingInterface_MultipleFrames)
    {
        auto* trackingInterface = ROS2::ROS2FrameTrackingInterface::Get();
        ASSERT_NE(trackingInterface, nullptr) << "ROS2FrameTrackingInterface not available";

        ROS2::ROS2FrameConfiguration config;

        std::vector<AZStd::unique_ptr<AZ::Entity>> entities;
        std::vector<const ROS2::ROS2FrameComponent*> frames;
        std::vector<AZStd::string> expectedFrameIds;

        constexpr int numOfEntities = 3;

        // Create and activate entities with frame components
        for (int i = 0; i < numOfEntities; i++)
        {
            entities.push_back(AZStd::make_unique<AZ::Entity>());
            entities[i]->SetName(AZStd::string::format("entity_%d", i));
            entities[i]->CreateComponent<AzFramework::TransformComponent>();
            auto frame = entities[i]->CreateComponent<ROS2::ROS2FrameComponent>(config);
            frames.push_back(frame);

            entities[i]->Init();
            entities[i]->Activate();

            expectedFrameIds.push_back(frame->GetNamespacedFrameID());
        }

        // Test frame count
        EXPECT_EQ(trackingInterface->GetRegisteredFrameCount(), numOfEntities);

        // Test that all frames are registered
        for (int i = 0; i < numOfEntities; i++)
        {
            EXPECT_TRUE(trackingInterface->IsFrameRegistered(entities[i]->GetId()));
        }

        // Test getting all registered frames
        const AZStd::unordered_set<AZ::EntityId>& registeredFrames = trackingInterface->GetRegisteredFrames();
        EXPECT_EQ(registeredFrames.size(), numOfEntities);

        for (int i = 0; i < numOfEntities; i++)
        {
            EXPECT_TRUE(registeredFrames.contains(entities[i]->GetId()));
        }

        // Test getting all namespaced frame IDs
        auto allFrameIds = trackingInterface->GetAllNamespacedFrameIds();
        EXPECT_EQ(allFrameIds.size(), numOfEntities);

        for (const auto& expectedId : expectedFrameIds)
        {
            EXPECT_TRUE(allFrameIds.contains(expectedId));
        }

        // Test bi-directional lookup (entity ID <-> namespaced frame ID)
        for (int i = 0; i < numOfEntities; i++)
        {
            // Get namespaced frame ID from entity
            AZStd::string namespacedFrameId = trackingInterface->GetNamespacedFrameId(entities[i]->GetId());
            EXPECT_EQ(namespacedFrameId, expectedFrameIds[i]);

            // Get entity from namespaced frame ID
            AZ::EntityId foundEntityId = trackingInterface->GetFrameEntityByNamespacedId(namespacedFrameId);
            EXPECT_EQ(foundEntityId, entities[i]->GetId());
        }
    }

    TEST_F(ROS2FrameComponentFixture, FrameTrackingInterface_FrameDeactivation)
    {
        auto* trackingInterface = ROS2::ROS2FrameTrackingInterface::Get();
        ASSERT_NE(trackingInterface, nullptr) << "ROS2FrameTrackingInterface not available";

        ROS2::ROS2FrameConfiguration config;

        AZ::Entity entity1, entity2;
        entity1.SetName("entity_1");
        entity2.SetName("entity_2");

        entity1.CreateComponent<AzFramework::TransformComponent>();
        entity2.CreateComponent<AzFramework::TransformComponent>();

        [[maybe_unused]] auto frame1 = entity1.CreateComponent<ROS2::ROS2FrameComponent>(config);
        [[maybe_unused]] auto frame2 = entity2.CreateComponent<ROS2::ROS2FrameComponent>(config);

        entity1.Init();
        entity2.Init();
        entity1.Activate();
        entity2.Activate();

        // Verify both frames are tracked
        EXPECT_EQ(trackingInterface->GetRegisteredFrameCount(), 2);

        // Deactivate one entity
        entity1.Deactivate();

        // Verify only one frame is tracked
        EXPECT_EQ(trackingInterface->GetRegisteredFrameCount(), 1);

        // Verify the deactivated frame is not registered
        EXPECT_FALSE(trackingInterface->IsFrameRegistered(entity1.GetId()));

        // Verify the active frame is still registered
        EXPECT_TRUE(trackingInterface->IsFrameRegistered(entity2.GetId()));
    }

    TEST_F(ROS2FrameComponentFixture, FrameTrackingInterface_InvalidQueries)
    {
        auto* trackingInterface = ROS2::ROS2FrameTrackingInterface::Get();
        ASSERT_NE(trackingInterface, nullptr) << "ROS2FrameTrackingInterface not available";

        // Test queries with invalid/non-existent entities
        AZ::EntityId invalidEntityId;

        // Test frame registration check with invalid entity
        EXPECT_FALSE(trackingInterface->IsFrameRegistered(invalidEntityId));

        // Test getting namespaced frame ID for invalid entity
        AZStd::string namespacedFrameId = trackingInterface->GetNamespacedFrameId(invalidEntityId);
        EXPECT_TRUE(namespacedFrameId.empty());

        // Test getting entity by non-existent namespaced frame ID
        AZ::EntityId foundEntityId = trackingInterface->GetFrameEntityByNamespacedId("non_existent_frame");
        EXPECT_FALSE(foundEntityId.IsValid());
    }

} // namespace UnitTest

// required to support running integration tests with Qt
AZTEST_EXPORT int AZ_UNIT_TEST_HOOK_NAME(int argc, char** argv)
{
    ::testing::InitGoogleMock(&argc, argv);
    AzQtComponents::PrepareQtPaths();
    QApplication app(argc, argv);
    AZ::Test::printUnusedParametersWarning(argc, argv);
    AZ::Test::addTestEnvironments({ new UnitTest::ROS2FrameComponentTestEnvironment() });
    int result = RUN_ALL_TESTS();
    return result;
}

IMPLEMENT_TEST_EXECUTABLE_MAIN();
