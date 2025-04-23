#include "AzCore/Component/Entity.h"
#include "AzCore/RTTI/ReflectContext.h"
#include "AzCore/Serialization/Json/BaseJsonSerializer.h"
#include "AzCore/Serialization/Json/JsonSerialization.h"
#include "AzCore/Serialization/SerializeContext.h"
#include "Odometry/ROS2WheelOdometry.h"
#include <AzCore/IO/Streamer/Streamer.h>
#include <AzCore/IO/Streamer/StreamerComponent.h>
#include <AzCore/Memory/Memory.h>
#include <AzCore/Memory/SystemAllocator.h>
#include <AzCore/UnitTest/TestTypes.h>
#include <AzTest/AzTest.h>

#include <AzCore/Serialization/Json/JsonSystemComponent.h>
#include <AzCore/UserSettings/UserSettingsComponent.h>
#include <AzTest/GemTestEnvironment.h>
#include <AzToolsFramework/UnitTest/ToolsTestApplication.h>

namespace UnitTest
{
    class ROS2SensorsTestEnvironment : public AZ::Test::GemTestEnvironment
    {
        // AZ::Test::GemTestEnvironment overrides ...
        void AddGemsAndComponents() override;
        AZ::ComponentApplication* CreateApplicationInstance() override;
        void PostSystemEntityActivate() override;

    public:
        ROS2SensorsTestEnvironment() = default;
        ~ROS2SensorsTestEnvironment() override = default;
    };

    void ROS2SensorsTestEnvironment::AddGemsAndComponents()
    {
        AddActiveGems(AZStd::to_array<AZStd::string_view>({ "ROS2", "ROS2Sensors"}));
        AddDynamicModulePaths({ "ROS2" });
        // AddComponentDescriptors(AZStd::initializer_list<AZ::ComponentDescriptor*>{ ROS2::ROS2WheelOdometryComponent::CreateDescriptor(),
                                                                                //    AZ::JsonSystemComponent::CreateDescriptor() });
        // AddRequiredComponents(AZStd::to_array<AZ::TypeId const>({ AZ::JsonSystemComponent::TYPEINFO_Uuid() }));
    }

    AZ::ComponentApplication* ROS2SensorsTestEnvironment::CreateApplicationInstance()
    {
        // Using ToolsTestApplication to have AzFramework and AzToolsFramework components.
        return aznew UnitTest::ToolsTestApplication("ROS2SensorsTestEnvironment");
    }

    void ROS2SensorsTestEnvironment::PostSystemEntityActivate()
    {
        AZ::UserSettingsComponentRequestBus::Broadcast(&AZ::UserSettingsComponentRequests::DisableSaveOnFinalize);
    }

    class ROS2SensorsTestFixture : public ::testing::Test
    {
    };

    // Smoke test
    TEST_F(ROS2SensorsTestFixture, WheelOdometryComponent)
    {
        // Create a WheelOdometryComponent and check if it is created successfully
        // ROS2::ROS2WheelOdometryComponent wheelOdometryComponent;
        // EXPECT_NE(wheelOdometryComponent, nullptr);
    }

    // Store test
    TEST_F(ROS2SensorsTestFixture, WheelOdometryComponentStore)
    {
        // // Create a WheelOdometryComponent and check if it is created successfully
        // ROS2::ROS2WheelOdometryComponent wheelOdometryComponent;
        // // EXPECT_NE(wheelOdometryComponent, nullptr);

        // // Create a JSON object to store the component's configuration
        // rapidjson::Document document;
        // document.SetObject();

        // AZ::Entity entity;
        // // auto* wheelOdometryComponent

        // auto jsonDocument = AZStd::make_unique<rapidjson::Document>();

        // AZ::JsonDeserializerSettings settings;
        // auto storeResult = AZ::JsonSerialization::Load(wheelOdometryComponent, *jsonDocument, settings);

        // // Check if the store operation was successful
        // EXPECT_EQ(storeResult.GetOutcome(), AZ::JsonSerializationResult::Outcomes::Success);
    }
} // namespace UnitTest

// required to support running integration tests with Qt and PhysX
AZTEST_EXPORT int AZ_UNIT_TEST_HOOK_NAME(int argc, char** argv)
{
    ::testing::InitGoogleMock(&argc, argv);
    AZ::Test::printUnusedParametersWarning(argc, argv);
    AZ::Test::addTestEnvironments({ new UnitTest::ROS2SensorsTestEnvironment() });
    int result = RUN_ALL_TESTS();
    return result;
}

IMPLEMENT_TEST_EXECUTABLE_MAIN();
