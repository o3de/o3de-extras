
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
#include <AzToolsFramework/Commands/PreemptiveUndoCache.h>
#include <AzToolsFramework/Entity/EditorEntityContextComponent.h>
#include <AzToolsFramework/ToolsComponents/TransformComponent.h>
#include <AzToolsFramework/UnitTest/AzToolsFrameworkTestHelpers.h>
#include <AzToolsFramework/UnitTest/ToolsTestApplication.h>
#include <ROS2/Frame/ROS2FrameComponent.h>
#include <ROS2/ROS2Bus.h>
#include <SystemComponents/ROS2SystemComponent.h>

#include <QApplication>
#include <gtest/gtest.h>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

namespace UnitTest
{
    class ROS2FrameComponentTestEnviroment : public AZ::Test::GemTestEnvironment
    {
        // AZ::Test::GemTestEnvironment overrides ...
        void AddGemsAndComponents() override;
        AZ::ComponentApplication* CreateApplicationInstance() override;
        void PostSystemEntityActivate() override;

    public:
        ROS2FrameComponentTestEnviroment() = default;
        ~ROS2FrameComponentTestEnviroment() override = default;
    };

    void ROS2FrameComponentTestEnviroment::AddGemsAndComponents()
    {
        AddActiveGems(AZStd::to_array<AZStd::string_view>({ "ROS2" }));
        AddDynamicModulePaths({});
        AddComponentDescriptors(AZStd::initializer_list<AZ::ComponentDescriptor*>{ ROS2::ROS2FrameComponent::CreateDescriptor(),
                                                                                   ROS2::ROS2SystemComponent::CreateDescriptor() });
        AddRequiredComponents(AZStd::to_array<AZ::TypeId const>({ ROS2::ROS2SystemComponent::TYPEINFO_Uuid() }));
    }

    AZ::ComponentApplication* ROS2FrameComponentTestEnviroment::CreateApplicationInstance()
    {
        // Using ToolsTestApplication to have AzFramework and AzToolsFramework components.
        return aznew UnitTest::ToolsTestApplication("ROS2FrameComponent");
    }

    void ROS2FrameComponentTestEnviroment::PostSystemEntityActivate()
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

        const std::string jointName(frame->GetJointName().GetCStr());
        const std::string frameId(frame->GetFrameID().c_str());

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
            const std::string jointName(frames[i]->GetJointName().GetCStr());
            const std::string frameId(frames[i]->GetFrameID().c_str());

            EXPECT_EQ(entities[i]->GetState(), AZ::Entity::State::Active);
            EXPECT_STRCASEEQ(jointName.c_str(), (entities[0]->GetName() + "/").c_str());
            EXPECT_STRCASEEQ(frameId.c_str(), (entities[0]->GetName() + "/sensor_frame").c_str());
        }
    }

} // namespace UnitTest

// required to support running integration tests with Qt and PhysX
AZTEST_EXPORT int AZ_UNIT_TEST_HOOK_NAME(int argc, char** argv)
{
    ::testing::InitGoogleMock(&argc, argv);
    AzQtComponents::PrepareQtPaths();
    QApplication app(argc, argv);
    AZ::Test::printUnusedParametersWarning(argc, argv);
    AZ::Test::addTestEnvironments({ new UnitTest::ROS2FrameComponentTestEnviroment() });
    int result = RUN_ALL_TESTS();
    return result;
}

IMPLEMENT_TEST_EXECUTABLE_MAIN();
