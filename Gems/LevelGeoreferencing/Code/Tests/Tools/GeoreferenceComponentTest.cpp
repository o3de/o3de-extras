
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

#include <QApplication>
#include <gtest/gtest.h>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include "Clients/GeoreferenceLevelComponent.h"
#include "Clients/GeoreferencingSystemComponent.h"
#include <Georeferencing/GeoreferenceBus.h>
namespace UnitTest
{
    constexpr float OneMillimeter = 0.001f;
    constexpr double MaxWGSError = 0.001 * (1.0 / 60.0 / 60.0); // 0.001 arc second
    class GeoreferenceComponentTestEnvironment : public AZ::Test::GemTestEnvironment
    {
        // AZ::Test::GemTestEnvironment overrides ...
        void AddGemsAndComponents() override;
        AZ::ComponentApplication* CreateApplicationInstance() override;
        void PostSystemEntityActivate() override;

    public:
        GeoreferenceComponentTestEnvironment() = default;
        ~GeoreferenceComponentTestEnvironment() override = default;
    };

    void GeoreferenceComponentTestEnvironment::AddGemsAndComponents()
    {
        AddActiveGems(AZStd::to_array<AZStd::string_view>({ "LevelGeoreferencing" }));
        AddDynamicModulePaths({});
        AddComponentDescriptors(AZStd::initializer_list<AZ::ComponentDescriptor*>{
            Georeferencing::GeoReferenceLevelComponent::CreateDescriptor(),
            Georeferencing::GeoreferencingSystemComponent::CreateDescriptor(),
        });
        AddRequiredComponents(AZStd::to_array<AZ::TypeId const>({ Georeferencing::GeoreferencingSystemComponent::TYPEINFO_Uuid() }));
    }

    AZ::ComponentApplication* GeoreferenceComponentTestEnvironment::CreateApplicationInstance()
    {
        // Using ToolsTestApplication to have AzFramework and AzToolsFramework components.
        return aznew UnitTest::ToolsTestApplication("GeoreferenceComponentTestEnvironment");
    }

    void GeoreferenceComponentTestEnvironment::PostSystemEntityActivate()
    {
        AZ::UserSettingsComponentRequestBus::Broadcast(&AZ::UserSettingsComponentRequests::DisableSaveOnFinalize);
    }

    class GeoreferenceComponentTestFixture : public ::testing::Test
    {
    protected:
        void SetUp() override;
        void TearDown() override;
        AZ::Entity* m_levelEntity;
        AZ::Entity* m_originEntity;
    };

    void GeoreferenceComponentTestFixture::SetUp()
    {
        m_levelEntity = aznew AZ::Entity();
        m_originEntity = aznew AZ::Entity();

        AZ_Assert(m_levelEntity, "Failed to create level entity");
        AZ_Assert(m_originEntity, "Failed to create origin entity");

        auto component = m_levelEntity->CreateComponent<Georeferencing::GeoReferenceLevelComponent>();
        AZ_Assert(component, "Failed to create GeoreferenceLevelComponent");

        auto transformComponent = m_originEntity->CreateComponent<AzFramework::TransformComponent>();
        AZ_Assert(transformComponent, "Failed to create TransformComponent");

        m_levelEntity->Init();
        m_originEntity->Init();
        m_levelEntity->Activate();
        m_originEntity->Activate();

        Georeferencing::GeoreferenceConfigurationRequestsBus::Broadcast(
            &Georeferencing::GeoreferenceConfigurationRequests::SetOriginEntity, m_originEntity->GetId());
    }

    void GeoreferenceComponentTestFixture::TearDown()
    {
        delete m_levelEntity;
        delete m_originEntity;
    }

    TEST_F(GeoreferenceComponentTestFixture, ComponentSmokeTest)
    {
        EXPECT_TRUE(m_levelEntity->GetState() == AZ::Entity::State::Active);
        EXPECT_TRUE(m_originEntity->GetState() == AZ::Entity::State::Active);
    }

    TEST_F(GeoreferenceComponentTestFixture, TestConfiguration)
    {
        // Set the origin entity as the origin of the level entity
        Georeferencing::GeoreferenceConfigurationRequestsBus::Broadcast(
            &Georeferencing::GeoreferenceConfigurationRequests::SetOriginEntity, m_originEntity->GetId());

        // Set origin coordinates
        Georeferencing::WGS::WGS84Coordinate originCoordinate;
        originCoordinate.SetAltitude(0.0);
        originCoordinate.SetLatitude(25.0);
        originCoordinate.SetAltitude(35.0);
        Georeferencing::GeoreferenceConfigurationRequestsBus::Broadcast(
            &Georeferencing::GeoreferenceConfigurationRequests::SetOriginCoordinates, originCoordinate);

        AZ::EntityId originEntityId;
        Georeferencing::GeoreferenceConfigurationRequestsBus::BroadcastResult(
            originEntityId, &Georeferencing::GeoreferenceConfigurationRequests::GetOriginEntity);
        EXPECT_TRUE(originEntityId == m_originEntity->GetId());

        Georeferencing::WGS::WGS84Coordinate originCoordinateResult;
        Georeferencing::GeoreferenceConfigurationRequestsBus::BroadcastResult(
            originCoordinateResult, &Georeferencing::GeoreferenceConfigurationRequests::GetOriginCoordinates);
        EXPECT_TRUE(originCoordinateResult == originCoordinate);
    }

    TEST_F(GeoreferenceComponentTestFixture, TestConversionToLevelAtOrigin)
    {
        // Set origin coordinates
        const Georeferencing::WGS::WGS84Coordinate originCoordinate{ 25.0, 35.0, 0.0 };
        Georeferencing::GeoreferenceConfigurationRequestsBus::Broadcast(
            &Georeferencing::GeoreferenceConfigurationRequests::SetOriginCoordinates, originCoordinate);

        // convert to level
        AZ::Vector3 levelCoordinate;
        Georeferencing::GeoreferenceRequestsBus::BroadcastResult(
            levelCoordinate, &Georeferencing::GeoreferenceRequests::ConvertFromWGS84ToLevel, originCoordinate);

        EXPECT_NEAR(levelCoordinate.GetX(), 0.0, OneMillimeter);
        EXPECT_NEAR(levelCoordinate.GetY(), 0.0, OneMillimeter);
        EXPECT_NEAR(levelCoordinate.GetZ(), 0.0, OneMillimeter);
    }

    TEST_F(GeoreferenceComponentTestFixture, TestConversionToWGS84AtOrigin)
    {
        // Set origin coordinates
        const Georeferencing::WGS::WGS84Coordinate originCoordinate{ 25.0, 35.0, 0.0 };
        Georeferencing::GeoreferenceConfigurationRequestsBus::Broadcast(
            &Georeferencing::GeoreferenceConfigurationRequests::SetOriginCoordinates, originCoordinate);

        Georeferencing::WGS::WGS84Coordinate resultCoordinate;
        Georeferencing::GeoreferenceRequestsBus::BroadcastResult(
            resultCoordinate, &Georeferencing::GeoreferenceRequests::ConvertFromLevelToWGS84, AZ::Vector3::CreateZero());

        EXPECT_NEAR(resultCoordinate.GetLatitude(), 25.0, MaxWGSError);
        EXPECT_NEAR(resultCoordinate.GetLongitude(), 35.0, MaxWGSError);
        EXPECT_NEAR(resultCoordinate.GetAltitude(), 0.0, MaxWGSError);
    }

    TEST_F(GeoreferenceComponentTestFixture, TestNavigationInDirectionENU)
    {
        // Set origin coordinates
        const Georeferencing::WGS::WGS84Coordinate originCoordinate{ 25.0, 35.0, 0.0 };

        Georeferencing::GeoreferenceConfigurationRequestsBus::Broadcast(
            &Georeferencing::GeoreferenceConfigurationRequests::SetOriginCoordinates, originCoordinate);

        const AZ::Vector3 north = AZ::Vector3::CreateAxisY(); // One meter north

        Georeferencing::WGS::WGS84Coordinate resultCoordinate;
        Georeferencing::GeoreferenceRequestsBus::BroadcastResult(
            resultCoordinate, &Georeferencing::GeoreferenceRequests::ConvertFromLevelToWGS84, north);

        EXPECT_GT(resultCoordinate.GetLatitude(), 25.0);
        EXPECT_NEAR(resultCoordinate.GetLongitude(), 35.0, MaxWGSError);
        EXPECT_NEAR(resultCoordinate.GetAltitude(), 0.0, MaxWGSError);
    }

    TEST_F(GeoreferenceComponentTestFixture, TestNavigationInDirectionRotatedENU)
    {
        // Set origin coordinates
        const Georeferencing::WGS::WGS84Coordinate originCoordinate{ 25.0, 35.0, 0.0 };

        Georeferencing::GeoreferenceConfigurationRequestsBus::Broadcast(
            &Georeferencing::GeoreferenceConfigurationRequests::SetOriginCoordinates, originCoordinate);

        // level is rotated 90 degrees around Z axis
        // positive X direction is of the level is now pointing east
        // negative Y direction is of the level is now pointing north
        const AZ::Transform transform = AZ::Transform::CreateRotationZ(AZ::DegToRad(90.0));
        AZ_Assert(m_originEntity, "No origin entity");
        AZ_Assert(m_originEntity->GetState() == AZ::Entity::State::Active, "Origin entity is not active");
        AZ::TransformBus::Event(m_originEntity->GetId(), &AZ::TransformBus::Events::SetWorldTM, transform);

        // Query is point straight north
        const Georeferencing::WGS::WGS84Coordinate queryCoordinate{ 25.001, 35.0, 0.0 };

        AZ::Vector3 resultLevel;
        Georeferencing::GeoreferenceRequestsBus::BroadcastResult(
            resultLevel, &Georeferencing::GeoreferenceRequests::ConvertFromWGS84ToLevel, queryCoordinate);
        EXPECT_LT(resultLevel.GetX(), 0.0);
        EXPECT_NEAR(resultLevel.GetY(), 0.0, OneMillimeter);
    }

} // namespace UnitTest

// required to support running integration tests with Qt and PhysX
AZTEST_EXPORT int AZ_UNIT_TEST_HOOK_NAME(int argc, char** argv)
{
    ::testing::InitGoogleMock(&argc, argv);
    AzQtComponents::PrepareQtPaths();
    QApplication app(argc, argv);
    AZ::Test::printUnusedParametersWarning(argc, argv);
    AZ::Test::addTestEnvironments({ new UnitTest::GeoreferenceComponentTestEnvironment() });
    int result = RUN_ALL_TESTS();
    return result;
}

IMPLEMENT_TEST_EXECUTABLE_MAIN();
