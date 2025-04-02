
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

#include <AzCore/Utils/Utils.h>
#include <AzFramework/Asset/AssetCatalog.h>
#include <AzFramework/Physics/Configuration/SystemConfiguration.h>
#include <AzFramework/Physics/PhysicsSystem.h>
#include <AzFramework/Physics/RigidBodyBus.h>
#include <AzFramework/Physics/SimulatedBodies/RigidBody.h>
#include <AzFramework/Physics/SystemBus.h>

namespace UnitTest
{
    class SimulationInterfaceTestEnvironment : public AZ::Test::GemTestEnvironment
    {
        // AZ::Test::GemTestEnvironment overrides ...
        void AddGemsAndComponents() override;
        AZ::ComponentApplication* CreateApplicationInstance() override;

    protected:
        void PostSystemEntityActivate() override;

    public:
        SimulationInterfaceTestEnvironment() = default;
        ~SimulationInterfaceTestEnvironment() override = default;
    };

    class SimulationInterfaceTestFixture
        : public ::testing::Test
        , protected Physics::DefaultWorldBus::Handler
    {
    protected:
        constexpr static auto PhysXRigidBodyComponentTypeId = "{D4E52A70-BDE1-4819-BD3C-93AB3F4F3BE3}"; // From PhysX
        constexpr static auto PhysXStaticBodyComponentTypeId = "{A2CCCD3D-FB31-4D65-8DCD-2CD7E1D09538}"; // From PhysX
        constexpr static auto PhysXShapeColliderComponentTypeId = "{30CC9E77-378C-49DF-9617-6BF191901FE0}"; // From PhysX
        constexpr static auto PhysXSphereColliderComponentTypeId = "{108CD341-E5C3-4AE1-B712-21E81ED6C277}"; // From PhysX
        constexpr static auto SphereShapeComponentTypeId = "{E24CBFF0-2531-4F8D-A8AB-47AF4D54BCD2}"; // From LmbrCentral

        void SetUp() override;
        void TearDown() override;

        AZ::EntityId CreateEntityWithStaticBodyComponent(const AZStd::string& entityName, const AZ::Transform& transform);

        void DeleteEntity(const AZ::EntityId& entityId);
        void ClearEntities();

        AZStd::unordered_map<AZ::EntityId, AZStd::unique_ptr<AZ::Entity>> m_entities;

        void AddAsset(const AZStd::string& assetPath);

        //! Ask the physics system to step forward in time
        void StepPhysics(int numSteps = 1);

        //! Ask the application to tick forward in time
        void TickApp(int numTicks = 1);

    private:
        AzPhysics::SceneHandle GetDefaultSceneHandle() const override;
        AzPhysics::Scene* m_defaultScene = nullptr;
        AzPhysics::SceneHandle m_testSceneHandle = AzPhysics::InvalidSceneHandle;
        AZStd::unordered_set<AZ::Data::AssetId> m_registeredAssets;
    };
} // namespace UnitTest