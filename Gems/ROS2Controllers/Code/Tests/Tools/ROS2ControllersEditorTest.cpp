/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <AzCore/UserSettings/UserSettingsComponent.h>
#include <AzTest/AzTest.h>
#include <AzTest/GemTestEnvironment.h>
#include <AzToolsFramework/UnitTest/ToolsTestApplication.h>

namespace UnitTest
{
    class ROS2ControllersTestEnvironment : public AZ::Test::GemTestEnvironment
    {
        // AZ::Test::GemTestEnvironment overrides ...
        void AddGemsAndComponents() override;
        AZ::ComponentApplication* CreateApplicationInstance() override;
        void PostSystemEntityActivate() override;

    public:
        ROS2ControllersTestEnvironment() = default;
        ~ROS2ControllersTestEnvironment() override = default;
    };

    void ROS2ControllersTestEnvironment::AddGemsAndComponents()
    {
        AddActiveGems(AZStd::to_array<AZStd::string_view>({ "ROS2", "ROS2Controllers" }));
        AddDynamicModulePaths({ "ROS2", "ROS2Controllers" });
    }

    AZ::ComponentApplication* ROS2ControllersTestEnvironment::CreateApplicationInstance()
    {
        // Using ToolsTestApplication to have AzFramework and AzToolsFramework components.
        return aznew UnitTest::ToolsTestApplication("ROS2ControllersTestEnvironment");
    }

    void ROS2ControllersTestEnvironment::PostSystemEntityActivate()
    {
        AZ::UserSettingsComponentRequestBus::Broadcast(&AZ::UserSettingsComponentRequests::DisableSaveOnFinalize);
    }
} // namespace UnitTest

AZTEST_EXPORT int AZ_UNIT_TEST_HOOK_NAME(int argc, char** argv)
{
    ::testing::InitGoogleMock(&argc, argv);
    AZ::Test::printUnusedParametersWarning(argc, argv);
    AZ::Test::addTestEnvironments({ new UnitTest::ROS2ControllersTestEnvironment() });
    int result = RUN_ALL_TESTS();
    return result;
}

IMPLEMENT_TEST_EXECUTABLE_MAIN();
