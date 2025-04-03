
/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */


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
#include <AzToolsFramework/UnitTest/ToolsTestApplication.h>


#include <QApplication>
#include <gtest/gtest.h>
#include <memory>
#include <string>
#include <string_view>
#include <vector>
#include <ROS2/ROS2Bus.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/publisher.hpp>
#include <std_msgs/msg/string.hpp>
namespace UnitTest
{
    class SimulationInterfaceROS2TestEnvironment : public AZ::Test::GemTestEnvironment
    {
        // AZ::Test::GemTestEnvironment overrides ...
        void AddGemsAndComponents() override;
        AZ::ComponentApplication* CreateApplicationInstance() override;
        void PostSystemEntityActivate() override;

    public:
        SimulationInterfaceROS2TestEnvironment() = default;
        ~SimulationInterfaceROS2TestEnvironment() override = default;
    };

    void SimulationInterfaceROS2TestEnvironment::AddGemsAndComponents()
    {
        AddActiveGems(AZStd::to_array<AZStd::string_view>({ "ROS2" }));

        AddDynamicModulePaths({"ROS2"});

    }

    AZ::ComponentApplication* SimulationInterfaceROS2TestEnvironment::CreateApplicationInstance()
    {
        // Using ToolsTestApplication to have AzFramework and AzToolsFramework components.
        return aznew UnitTest::ToolsTestApplication("SimulationInterfaceROS2TestEnvironment");
    }

    void SimulationInterfaceROS2TestEnvironment::PostSystemEntityActivate()
    {
        AZ::UserSettingsComponentRequestBus::Broadcast(&AZ::UserSettingsComponentRequests::DisableSaveOnFinalize);
    }

    class SimulationInterfaceROS2TestFixture : public ::testing::Test
    {
    protected:

    };

    TEST_F(SimulationInterfaceROS2TestFixture, TestIfROS2NodeIsAvailable)
    {
        auto interface = ROS2::ROS2Interface::Get();
        ASSERT_TRUE(interface) << "ROS2 interface is not available.";
        auto node = interface->GetNode();
        ASSERT_TRUE(node) << "Node is not available.";

    }

    TEST_F(SimulationInterfaceROS2TestFixture, TestIfROS2NodeIsAvailable2)
    {
        auto interface = ROS2::ROS2Interface::Get();
        ASSERT_TRUE(interface) << "ROS2 interface is not available.";
        auto node = interface->GetNode();
        ASSERT_TRUE(node) << "Node is not available.";
        auto pub = node->create_publisher<std_msgs::msg::String >("hello", 10);
        std_msgs::msg::String msg;
        for (int i = 0; i < 1000000; i++)
        {
            msg.data = "Hello World!";
            pub->publish(msg);
            AZ::ComponentApplication* app = nullptr;
            AZ::ComponentApplicationBus::BroadcastResult(app, &AZ::ComponentApplicationBus::Events::GetApplication);
            app->Tick();
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
    AZ::Test::addTestEnvironments({ new UnitTest::SimulationInterfaceROS2TestEnvironment() });
    int result = RUN_ALL_TESTS();
    return result;
}

IMPLEMENT_TEST_EXECUTABLE_MAIN();
