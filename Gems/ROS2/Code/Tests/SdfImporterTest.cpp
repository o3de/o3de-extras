/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <AzCore/UserSettings/UserSettingsComponent.h>
#include <AzToolsFramework/UnitTest/ToolsTestApplication.h>

#include <QApplication>

#include <AzCore/Component/ComponentApplicationBus.h>
#include <AzCore/Component/Entity.h>
#include <AzCore/RTTI/RTTIMacros.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/Slice/SliceAssetHandler.h>
#include <AzQtComponents/Utilities/QtPluginPaths.h>
#include <AzTest/GemTestEnvironment.h>
#include <AzToolsFramework/Commands/PreemptiveUndoCache.h>
#include <AzToolsFramework/Entity/EditorEntityContextComponent.h>
#include <AzToolsFramework/ToolsComponents/TransformComponent.h>

#include <Camera/ROS2CameraSensorEditorComponent.h>
#include <ROS2/Frame/ROS2FrameComponent.h>
#include <RobotImporter/SDFormat/Parser.h>
#include <RobotImporter/SDFormat/SensorsMaker.h>

#include "SdfModel.h"

#include <sdf/Camera.hh>
#include <sdf/Link.hh>
#include <sdf/Sensor.hh>

namespace UnitTest
{
    class SDFormatImporterPhysicsTestEnvironment : public AZ::Test::GemTestEnvironment
    {
        // AZ::Test::GemTestEnvironment overrides ...
        void AddGemsAndComponents() override;
        AZ::ComponentApplication* CreateApplicationInstance() override;
        void PostSystemEntityActivate() override;

    public:
        SDFormatImporterPhysicsTestEnvironment() = default;
        ~SDFormatImporterPhysicsTestEnvironment() override = default;
    };

    void SDFormatImporterPhysicsTestEnvironment::AddGemsAndComponents()
    {
        AddDynamicModulePaths({ "PhysX.Editor.Gem" });
        AddComponentDescriptors({});
    }

    AZ::ComponentApplication* SDFormatImporterPhysicsTestEnvironment::CreateApplicationInstance()
    {
        // Using ToolsTestApplication to have AzFramework and AzToolsFramework components.
        return aznew UnitTest::ToolsTestApplication("UrdfImpoterPhysics");
    }

    void SDFormatImporterPhysicsTestEnvironment::PostSystemEntityActivate()
    {
        AZ::UserSettingsComponentRequestBus::Broadcast(&AZ::UserSettingsComponentRequests::DisableSaveOnFinalize);
    }

    class SDFormatImportPhysicsFixture : public ::testing::Test
    {
    };

    TEST_F(SDFormatImportPhysicsFixture, EmptyCameraSensor)
    {
        AZ::Entity entity;
        entity.CreateComponent<AzToolsFramework::Components::TransformComponent>();
        entity.Init();

        sdf::Link link;
        sdf::Sensor sensor;
        sdf::Camera cameraSensor;
        sensor.SetType(sdf::SensorType::CAMERA);
        sensor.SetCameraSensor(cameraSensor);
        link.AddSensor(sensor);

        ROS2::SDFormat::SensorsMaker sensorMaker;
        sensorMaker.AddSensors(entity.GetId(), &link);

        const auto& components = entity.GetComponents();
        EXPECT_EQ(components.size(), 3U);
        EXPECT_EQ(azrtti_typeid<ROS2::ROS2FrameComponent>(), azrtti_typeid(components[1]));
        EXPECT_EQ(azrtti_typeid<ROS2::ROS2CameraSensorEditorComponent>(), azrtti_typeid(components[2]));
    }

    TEST_F(SDFormatImportPhysicsFixture, SDFormatCameraOnlySensor)
    {
        AZ::Entity entity;
        entity.CreateComponent<AzToolsFramework::Components::TransformComponent>();
        entity.Init();

        const auto xmlStr = SdfModel::GetSdfWithOneLink();
        const auto sdfRoot = ROS2::SDFormat::Parser::Parse(xmlStr);
        const auto* sdfModel = sdfRoot->Model();
        ASSERT_TRUE(sdfModel);

        ROS2::SDFormat::SensorsMaker sensorMaker;
        for (size_t li = 0; li < sdfModel->LinkCount(); ++li)
        {
            const auto* link = sdfModel->LinkByIndex(li);
            sensorMaker.AddSensors(entity.GetId(), link);
        }

        const auto& components = entity.GetComponents();
        EXPECT_EQ(components.size(), 3U);
        EXPECT_EQ(azrtti_typeid<ROS2::ROS2FrameComponent>(), azrtti_typeid(components[1]));
        EXPECT_EQ(azrtti_typeid<ROS2::ROS2CameraSensorEditorComponent>(), azrtti_typeid(components[2]));
    }

    TEST_F(SDFormatImportPhysicsFixture, SDFormatThreeSensors)
    {
        AZ::Entity entity;
        entity.CreateComponent<AzToolsFramework::Components::TransformComponent>();
        entity.Init();

        const auto xmlStr = SdfModel::GetSdfWithTwoLinksAndJoint();
        const auto sdfRoot = ROS2::SDFormat::Parser::Parse(xmlStr);
        const auto* sdfModel = sdfRoot->Model();
        ASSERT_TRUE(sdfModel);

        ROS2::SDFormat::SensorsMaker sensorMaker;
        for (size_t li = 0; li < sdfModel->LinkCount(); ++li)
        {
            const auto* link = sdfModel->LinkByIndex(li);
            sensorMaker.AddSensors(entity.GetId(), link);
        }

        const auto& components = entity.GetComponents();
        EXPECT_EQ(components.size(), 3U);
        EXPECT_EQ(azrtti_typeid<ROS2::ROS2FrameComponent>(), azrtti_typeid(components[1]));
        EXPECT_EQ(azrtti_typeid<ROS2::ROS2CameraSensorEditorComponent>(), azrtti_typeid(components[2]));
    }

} // namespace UnitTest

// required to support running integration tests with Qt and PhysX
AZTEST_EXPORT int AZ_UNIT_TEST_HOOK_NAME(int argc, char** argv)
{
    ::testing::InitGoogleMock(&argc, argv);
    AzQtComponents::PrepareQtPaths();
    QApplication app(argc, argv);
    AZ::Test::printUnusedParametersWarning(argc, argv);
    AZ::Test::addTestEnvironments({ new UnitTest::SDFormatImporterPhysicsTestEnvironment() });
    int result = RUN_ALL_TESTS();
    return result;
}

IMPLEMENT_TEST_EXECUTABLE_MAIN();
