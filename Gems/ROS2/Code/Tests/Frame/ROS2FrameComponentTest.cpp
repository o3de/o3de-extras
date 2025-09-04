
/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "../../../../../../../engine/o3de/Code/Framework/AzCore/AzCore/std/typetraits/has_virtual_destructor.h"
#include "Frame/NamespaceComputation.h"
#include "Frame/ROS2FrameGameSystemComponent.h"
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
#include <QApplication>
#include <ROS2/Frame/ROS2FrameComponent.h>
#include <ROS2/Frame/ROS2FrameComponentBus.h>
#include <ROS2/Frame/ROS2FrameTrackingInterface.h>
#include <ROS2/ROS2Bus.h>
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
        AddComponentDescriptors(
            AZStd::initializer_list<AZ::ComponentDescriptor*>{ ROS2::ROS2FrameComponent::CreateDescriptor(),
                                                               ROS2::ROS2SystemComponent::CreateDescriptor(),
                                                               ROS2::ROS2FrameGameSystemComponent::CreateDescriptor() });
        AddRequiredComponents(AZStd::to_array<AZ::TypeId const>(
            { ROS2::ROS2SystemComponent::TYPEINFO_Uuid(), ROS2::ROS2FrameGameSystemComponent::TYPEINFO_Uuid() }));
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

    // Simple test, default configuration, no hierarchy, no namespace change
    TEST_F(ROS2FrameComponentFixture, TestNamespaceResolvementDefault)
    {
        AZStd::vector<AZStd::pair<AZStd::string, ROS2::ROS2FrameConfiguration>> configurations;
        ROS2::ROS2FrameConfiguration config;
        config.m_namespaceConfiguration.m_namespaceStrategy = ROS2::NamespaceConfiguration::NamespaceStrategy::Default;
        configurations.emplace_back("root", config);
        const auto namespaceName = ROS2::ComputeNamespace(configurations);
        EXPECT_STREQ(namespaceName.c_str(), "root");
    }

    TEST_F(ROS2FrameComponentFixture, TestNamespaceResolvementCustom)
    {
        AZStd::vector<AZStd::pair<AZStd::string, ROS2::ROS2FrameConfiguration>> configurations;
        ROS2::ROS2FrameConfiguration config;
        config.m_namespaceConfiguration.m_namespaceStrategy = ROS2::NamespaceConfiguration::NamespaceStrategy::Custom;
        config.m_namespaceConfiguration.m_customNamespace = "foo";
        configurations.emplace_back("root", config);
        const auto namespaceName = ROS2::ComputeNamespace(configurations);
        EXPECT_STREQ(namespaceName.c_str(), "foo");
    }

    TEST_F(ROS2FrameComponentFixture, TestNamespaceResolvementHierachy1)
    {
        AZStd::vector<AZStd::pair<AZStd::string, ROS2::ROS2FrameConfiguration>> configurations;
        ROS2::ROS2FrameConfiguration config;
        config.m_namespaceConfiguration.m_namespaceStrategy = ROS2::NamespaceConfiguration::NamespaceStrategy::Default;
        configurations.emplace_back("child1", config);
        configurations.emplace_back("root", config);
        const auto namespaceName = ROS2::ComputeNamespace(configurations);
        EXPECT_STREQ(namespaceName.c_str(), "root");
    }

    TEST_F(ROS2FrameComponentFixture, TestNamespaceResolvementHierachy2)
    {
        AZStd::vector<AZStd::pair<AZStd::string, ROS2::ROS2FrameConfiguration>> configurations;
        ROS2::ROS2FrameConfiguration config;
        config.m_namespaceConfiguration.m_namespaceStrategy = ROS2::NamespaceConfiguration::NamespaceStrategy::Custom;
        config.m_namespaceConfiguration.m_customNamespace = "foo";
        configurations.emplace_back("child1", config);
        config.m_namespaceConfiguration.m_namespaceStrategy = ROS2::NamespaceConfiguration::NamespaceStrategy::Default;
        configurations.emplace_back("root", config);
        const auto namespaceName = ROS2::ComputeNamespace(configurations);
        EXPECT_STREQ(namespaceName.c_str(), "root/foo");
    }

    TEST_F(ROS2FrameComponentFixture, TestNamespaceResolvementHierachy4)
    {
        AZStd::vector<AZStd::pair<AZStd::string, ROS2::ROS2FrameConfiguration>> configurations;
        ROS2::ROS2FrameConfiguration config;
        config.m_namespaceConfiguration.m_namespaceStrategy = ROS2::NamespaceConfiguration::NamespaceStrategy::Custom;
        config.m_namespaceConfiguration.m_customNamespace = "bar";
        configurations.emplace_back("child2", config);
        config.m_namespaceConfiguration.m_namespaceStrategy = ROS2::NamespaceConfiguration::NamespaceStrategy::Custom;
        config.m_namespaceConfiguration.m_customNamespace = "foo";
        configurations.emplace_back("child1", config);
        config.m_namespaceConfiguration.m_namespaceStrategy = ROS2::NamespaceConfiguration::NamespaceStrategy::Default;
        configurations.emplace_back("root", config);
        const auto namespaceName = ROS2::ComputeNamespace(configurations);
        EXPECT_STREQ(namespaceName.c_str(), "root/foo/bar");
    }

    TEST_F(ROS2FrameComponentFixture, SingleFrameDefault)
    {
        ROS2::ROS2FrameConfiguration config;

        AZ::Entity entity;
        const std::string entityName = entity.GetName().c_str();
        entity.CreateComponent<AzFramework::TransformComponent>();
        auto frame = entity.CreateComponent<ROS2::ROS2FrameComponent>(config);

        entity.Init();
        entity.Activate();

        const std::string jointName(frame->GetNamespacedJointName().c_str());
        const std::string frameId(frame->GetNamespacedFrameID().c_str());

        EXPECT_EQ(entity.GetState(), AZ::Entity::State::Active);
        EXPECT_STRCASEEQ(jointName.c_str(), (entityName + "/").c_str());
        EXPECT_STRCASEEQ(frameId.c_str(), (entityName + "/sensor_frame").c_str());
    }

    TEST_F(ROS2FrameComponentFixture, ThreeFramesDefault)
    {
        ROS2::ROS2FrameConfiguration config;

        AZStd::vector<AZStd::unique_ptr<AZ::Entity>> entities;
        AZStd::vector<const ROS2::ROS2FrameComponent*> frames;

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
            const std::string jointName(frames[i]->GetNamespacedJointName().c_str());
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

        auto rosifiedName = entityName;
        ASSERT_STREQ(frame->GetNamespace().c_str(), rosifiedName.c_str());

        // Note that namespace parameter is only applied using Custom strategy
        {
            auto configuration = frame->GetConfiguration();
            entity.Deactivate();
            configuration.m_namespaceConfiguration.m_customNamespace = "MyCustomNamespace";
            configuration.m_namespaceConfiguration.m_namespaceStrategy = ROS2::NamespaceConfiguration::NamespaceStrategy::Custom;
            frame->SetConfiguration(configuration);
            entity.Activate();
            EXPECT_STREQ(frame->GetNamespace().c_str(), "MyCustomNamespace");
        }

        // Empty strategy clears the namespace
        {
            auto configuration = frame->GetConfiguration();
            configuration.m_namespaceConfiguration.m_namespaceStrategy = ROS2::NamespaceConfiguration::NamespaceStrategy::Empty;
            entity.Deactivate();
            frame->SetConfiguration(configuration);
            entity.Activate();
            EXPECT_STREQ(frame->GetNamespace().c_str(), "");
        }
        // From Entity Name strategy uses rosified version of Entity Name
        {
            auto configuration = frame->GetConfiguration();
            configuration.m_namespaceConfiguration.m_namespaceStrategy = ROS2::NamespaceConfiguration::NamespaceStrategy::FromEntityName;
            entity.Deactivate();
            frame->SetConfiguration(configuration);
            entity.Activate();
            EXPECT_STREQ(frame->GetNamespace().c_str(), rosifiedName.c_str());
        }
        // Default strategy is like From Entity Name strategy if the entity is on top of hierarchy ...
        {
            auto configuration = frame->GetConfiguration();
            configuration.m_namespaceConfiguration.m_namespaceStrategy = ROS2::NamespaceConfiguration::NamespaceStrategy::Default;
            entity.Deactivate();
            frame->SetConfiguration(configuration);
            entity.Activate();
            EXPECT_STREQ(frame->GetNamespace().c_str(), rosifiedName.c_str());
        }

        AZ::Entity newRootEntity;
        AZStd::string newRootName = "new_root";
        newRootEntity.SetName(newRootName);
        newRootEntity.CreateComponent<AzFramework::TransformComponent>();
        newRootEntity.CreateComponent<ROS2::ROS2FrameComponent>(config);
        newRootEntity.Init();
        newRootEntity.Activate();
        AZ::TransformBus::Event(entity.GetId(), &AZ::TransformBus::Events::SetParent, newRootEntity.GetId());

        // If there is a parent Default strategy concatenates parent's namespace with rosified name
        {
            auto configuration = frame->GetConfiguration();
            configuration.m_namespaceConfiguration.m_namespaceStrategy = ROS2::NamespaceConfiguration::NamespaceStrategy::Default;
            entity.Deactivate();
            frame->SetConfiguration(configuration);
            entity.Activate();
            EXPECT_STREQ(frame->GetNamespace().c_str(), newRootName.c_str());
        }
    }

    TEST_F(ROS2FrameComponentFixture, ComponentBusInterface)
    {


        ROS2::ROS2FrameConfiguration config;
        config.m_frameName = "test_frame";
        config.m_jointName = "test_joint";

        AZ::Entity entity;
        entity.SetName("test_entity");
        entity.CreateComponent<AzFramework::TransformComponent>();
        entity.CreateComponent<ROS2::ROS2FrameComponent>(config);

        entity.Init();
        entity.Activate();

        // Test component bus interface
        AZStd::string namespace_result;
        AZStd::string frameId_result;
        AZStd::string jointName_result;
        AZStd::string jointNameRaw_result;
        AZStd::string frameName_result;
        AZStd::string globalFrameName_result;

        ROS2::ROS2FrameComponentBus::EventResult(namespace_result, entity.GetId(), &ROS2::ROS2FrameComponentRequests::GetNamespace);
        ROS2::ROS2FrameComponentBus::EventResult(frameId_result, entity.GetId(), &ROS2::ROS2FrameComponentRequests::GetNamespacedFrameID);
        ROS2::ROS2FrameComponentBus::EventResult(
            jointName_result, entity.GetId(), &ROS2::ROS2FrameComponentRequests::GetNamespacedJointName);
        ROS2::ROS2FrameComponentBus::EventResult(jointNameRaw_result, entity.GetId(), &ROS2::ROS2FrameComponentRequests::GetJointName);
        ROS2::ROS2FrameComponentBus::EventResult(frameName_result, entity.GetId(), &ROS2::ROS2FrameComponentRequests::GetFrameName);
        ROS2::ROS2FrameComponentBus::EventResult(
            globalFrameName_result, entity.GetId(), &ROS2::ROS2FrameComponentRequests::GetGlobalFrameID);

        const auto globalNamespace = ROS2::GetGlobalFrameIDFromRegistry();

        EXPECT_EQ(namespace_result, "test_entity");
        EXPECT_EQ(frameId_result, "test_entity/test_frame");
        EXPECT_EQ(jointName_result, "test_entity/test_joint");
        EXPECT_EQ(jointNameRaw_result, "test_joint");
        EXPECT_EQ(frameName_result, "test_frame");
        EXPECT_EQ(globalFrameName_result, "test_entity/"+globalNamespace); // Uses default global frame name
    }

    TEST_F(ROS2FrameComponentFixture, FrameTrackingInterfaceBasic)
    {
        auto trackingInterface = ROS2::ROS2FrameTrackingInterface::Get();
        ASSERT_NE(trackingInterface, nullptr);

        // Initially no frames should be registered
        EXPECT_EQ(trackingInterface->GetRegisteredFrameCount(), 0);
        EXPECT_TRUE(trackingInterface->GetRegisteredFrames().empty());
        EXPECT_TRUE(trackingInterface->GetAllNamespacedFrameIds().empty());

        // Create and activate a frame component
        ROS2::ROS2FrameConfiguration config;
        config.m_frameName = "tracked_frame";

        AZ::Entity entity;
        entity.SetName("tracked_entity");
        entity.CreateComponent<AzFramework::TransformComponent>();
        entity.CreateComponent<ROS2::ROS2FrameComponent>(config);

        entity.Init();
        entity.Activate();

        // Frame should now be registered
        EXPECT_EQ(trackingInterface->GetRegisteredFrameCount(), 1);
        EXPECT_TRUE(trackingInterface->IsFrameRegistered(entity.GetId()));
        EXPECT_FALSE(trackingInterface->GetRegisteredFrames().empty());
        EXPECT_EQ(trackingInterface->GetRegisteredFrames().size(), 1);
        EXPECT_TRUE(trackingInterface->GetRegisteredFrames().contains(entity.GetId()));

        // Check namespaced frame ID lookup
        auto namespacedFrameId = trackingInterface->GetNamespacedFrameId(entity.GetId());
        ASSERT_TRUE(namespacedFrameId.size() > 0);
        EXPECT_STREQ(namespacedFrameId.c_str(), "tracked_entity/tracked_frame");

        auto allFrameIds = trackingInterface->GetAllNamespacedFrameIds();
        EXPECT_EQ(allFrameIds.size(), 1);
        EXPECT_TRUE(allFrameIds.contains("tracked_entity/tracked_frame"));

        // Check reverse lookup
        auto foundEntityId = trackingInterface->GetFrameEntityByNamespacedId("tracked_entity/tracked_frame");
        ASSERT_TRUE(foundEntityId.IsValid());
        EXPECT_EQ(foundEntityId, entity.GetId());

        // Deactivate entity and check frame is unregistered
        entity.Deactivate();

        EXPECT_EQ(trackingInterface->GetRegisteredFrameCount(), 0);
        EXPECT_FALSE(trackingInterface->IsFrameRegistered(entity.GetId()));
        EXPECT_TRUE(trackingInterface->GetRegisteredFrames().empty());
        EXPECT_TRUE(trackingInterface->GetAllNamespacedFrameIds().empty());
    }

    TEST_F(ROS2FrameComponentFixture, FrameTrackingInterfaceMultipleFrames)
    {
        auto trackingInterface = ROS2::ROS2FrameTrackingInterface::Get();
        ASSERT_NE(trackingInterface, nullptr);

        // Create multiple frame entities
        constexpr int numFrames = 3;
        AZStd::vector<AZStd::unique_ptr<AZ::Entity>> entities;
        AZStd::vector<AZStd::string> expectedFrameIds;

        for (int i = 0; i < numFrames; ++i)
        {
            ROS2::ROS2FrameConfiguration config;
            config.m_frameName = AZStd::string::format("frame_%d", i);

            auto entity = AZStd::make_unique<AZ::Entity>();
            entity->SetName(AZStd::string::format("entity_%d", i));
            entity->CreateComponent<AzFramework::TransformComponent>();
            entity->CreateComponent<ROS2::ROS2FrameComponent>(config);

            expectedFrameIds.push_back(AZStd::string::format("entity_%d/frame_%d", i, i));

            entity->Init();
            entity->Activate();

            entities.push_back(AZStd::move(entity));
        }

        // Check all frames are registered
        EXPECT_EQ(trackingInterface->GetRegisteredFrameCount(), numFrames);
        EXPECT_EQ(trackingInterface->GetRegisteredFrames().size(), numFrames);
        EXPECT_EQ(trackingInterface->GetAllNamespacedFrameIds().size(), numFrames);

        // Check each frame individually
        for (int i = 0; i < numFrames; ++i)
        {
            EXPECT_TRUE(trackingInterface->IsFrameRegistered(entities[i]->GetId()));

            auto namespacedFrameId = trackingInterface->GetNamespacedFrameId(entities[i]->GetId());
            ASSERT_TRUE(namespacedFrameId.size()>0);
            EXPECT_EQ(namespacedFrameId, expectedFrameIds[i]);

            auto foundEntityId = trackingInterface->GetFrameEntityByNamespacedId(expectedFrameIds[i]);
            ASSERT_TRUE(foundEntityId.IsValid());
            EXPECT_EQ(foundEntityId, entities[i]->GetId());
        }

        // Deactivate one frame and check tracking updates
        entities[1]->Deactivate();
        EXPECT_EQ(trackingInterface->GetRegisteredFrameCount(), numFrames - 1);
        EXPECT_FALSE(trackingInterface->IsFrameRegistered(entities[1]->GetId()));
        EXPECT_TRUE(trackingInterface->IsFrameRegistered(entities[0]->GetId()));
        EXPECT_TRUE(trackingInterface->IsFrameRegistered(entities[2]->GetId()));

        // Deactivate remaining frames
        entities[0]->Deactivate();
        entities[2]->Deactivate();
        EXPECT_EQ(trackingInterface->GetRegisteredFrameCount(), 0);
        EXPECT_TRUE(trackingInterface->GetRegisteredFrames().empty());
    }

    TEST_F(ROS2FrameComponentFixture, FrameTrackingInterfaceEdgeCases)
    {
        auto trackingInterface = ROS2::ROS2FrameTrackingInterface::Get();
        ASSERT_NE(trackingInterface, nullptr);

        // Test with invalid entity ID
        AZ::EntityId invalidEntityId;
        EXPECT_FALSE(trackingInterface->IsFrameRegistered(invalidEntityId));

        auto invalidNamespacedFrameId = trackingInterface->GetNamespacedFrameId(invalidEntityId);
        EXPECT_FALSE(invalidNamespacedFrameId.size()>0);

        // Test lookup with non-existent namespaced frame ID
        auto invalidEntityLookup = trackingInterface->GetFrameEntityByNamespacedId("non_existent/frame");
        EXPECT_FALSE(invalidEntityLookup.IsValid());

        // Test with entity that doesn't have a frame component
        AZ::Entity entityWithoutFrame;
        entityWithoutFrame.CreateComponent<AzFramework::TransformComponent>();
        entityWithoutFrame.Init();
        entityWithoutFrame.Activate();

        EXPECT_FALSE(trackingInterface->IsFrameRegistered(entityWithoutFrame.GetId()));

        auto noFrameNamespacedId = trackingInterface->GetNamespacedFrameId(entityWithoutFrame.GetId());
        EXPECT_FALSE(noFrameNamespacedId.size()>0);
    }

    TEST_F(ROS2FrameComponentFixture, FrameTrackingInterfaceNamespaceChanges)
    {
        auto trackingInterface = ROS2::ROS2FrameTrackingInterface::Get();
        ASSERT_NE(trackingInterface, nullptr);

        ROS2::ROS2FrameConfiguration config;
        config.m_frameName = "dynamic_frame";
        config.m_namespaceConfiguration.m_namespaceStrategy = ROS2::NamespaceConfiguration::NamespaceStrategy::Custom;
        config.m_namespaceConfiguration.m_customNamespace = "initial_namespace";

        AZ::Entity entity;
        entity.SetName("dynamic_entity");
        entity.CreateComponent<AzFramework::TransformComponent>();
        auto frame = entity.CreateComponent<ROS2::ROS2FrameComponent>(config);

        entity.Init();
        entity.Activate();

        // Check initial registration
        EXPECT_TRUE(trackingInterface->IsFrameRegistered(entity.GetId()));
        auto initialFrameId = trackingInterface->GetNamespacedFrameId(entity.GetId());
        ASSERT_TRUE(initialFrameId.size()>0);
        EXPECT_EQ(initialFrameId, "initial_namespace/dynamic_frame");

        // Change namespace configuration
        auto newConfig = frame->GetConfiguration();
        newConfig.m_namespaceConfiguration.m_customNamespace = "updated_namespace";

        entity.Deactivate();
        frame->SetConfiguration(newConfig);
        entity.Activate();

        // Check updated registration
        EXPECT_TRUE(trackingInterface->IsFrameRegistered(entity.GetId()));
        auto updatedFrameId = trackingInterface->GetNamespacedFrameId(entity.GetId());
        ASSERT_TRUE(updatedFrameId.size()>0);
        EXPECT_EQ(updatedFrameId, "updated_namespace/dynamic_frame");

        // Old frame ID should not exist anymore
        auto oldEntityLookup = trackingInterface->GetFrameEntityByNamespacedId("initial_namespace/dynamic_frame");
        EXPECT_FALSE(oldEntityLookup.IsValid());

        // New frame ID should work
        auto newEntityLookup = trackingInterface->GetFrameEntityByNamespacedId("updated_namespace/dynamic_frame");
        ASSERT_TRUE(newEntityLookup.IsValid());
        EXPECT_EQ(newEntityLookup, entity.GetId());
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
