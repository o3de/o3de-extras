
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

#include <Clients/ROS2SimulationInterfacesSystemComponent.h>
#include <ROS2/ROS2Bus.h>
#include <Services/DeleteEntityServiceHandler.h>
#include <Services/GetEntitiesServiceHandler.h>
#include <Services/GetEntitiesStatesServiceHandler.h>
#include <Services/GetEntityStateServiceHandler.h>
#include <Services/GetSimulatorFeaturesServiceHandler.h>
#include <Services/GetSimulationStateServiceHandler.h>
#include <Services/GetSpawnablesServiceHandler.h>
#include <Services/ROS2ServiceBase.h>
#include <Services/ResetSimulationServiceHandler.h>
#include <Services/SetEntityStateServiceHandler.h>
#include <Services/SetSimulationStateServiceHandler.h>
#include <Services/SpawnEntityServiceHandler.h>
#include <Services/StepSimulationServiceHandler.h>

#include "Mocks/SimulationEntityManagerMock.h"
#include "Mocks/SimulationFeaturesAggregatorRequestsHandlerMock.h"
#include "Mocks/SimulationManagerMock.h"

#include <QApplication>

#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/create_client.hpp>
#include <simulation_interfaces/action/simulate_steps.hpp>
#include <simulation_interfaces/msg/simulator_features.hpp>
#include <std_msgs/msg/string.hpp>

#include <gtest/gtest.h>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

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
        using namespace ROS2SimulationInterfaces;
        AddActiveGems(AZStd::to_array<AZStd::string_view>({ "ROS2" }));
        AddDynamicModulePaths({ "ROS2" });
        AddComponentDescriptors(AZStd::initializer_list<AZ::ComponentDescriptor*>{
            ROS2SimulationInterfacesSystemComponent::CreateDescriptor(),
        });
        AddRequiredComponents(AZStd::to_array<AZ::TypeId const>({ ROS2SimulationInterfacesSystemComponent::TYPEINFO_Uuid() }));
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
        std::shared_ptr<rclcpp::Node> GetRos2Node()
        {
            auto interface = ROS2::ROS2Interface::Get();
            AZ_Assert(interface, "ROS2 interface is not available.");
            auto node = interface->GetNode();
            AZ_Assert(node, "Node is not available.");
            return node;
        }
        void SpinAppSome(int numberOfTicks = 50)
        {
            AZ::ComponentApplication* app = nullptr;
            AZ::ComponentApplicationBus::BroadcastResult(app, &AZ::ComponentApplicationBus::Events::GetApplication);
            AZ_Assert(app, "Application pointer is not available.");
            for (int i = 0; i < numberOfTicks; ++i)
            {
                app->Tick();
            }
        }

        template<typename Future>
        void SpinAppUntilFuture(Future& future)
        {
            AZ::ComponentApplication* app = nullptr;
            AZ::ComponentApplicationBus::BroadcastResult(app, &AZ::ComponentApplicationBus::Events::GetApplication);
            AZ_Assert(app, "Application pointer is not available.");
            constexpr int MaximumTicks = 1000;
            for (int i = 0; i < MaximumTicks; ++i)
            {
                app->Tick();
                if (future.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
                {
                    return;
                }
            }
        }
    };

    //! Perform a smoke test to check if the ROS2 node is available from ROS2 gem
    TEST_F(SimulationInterfaceROS2TestFixture, TestIfROS2NodeIsAvailable)
    {
        ASSERT_NE(GetRos2Node(), nullptr) << "ROS2 node is not available.";
    }

    //! Perform a smoke test to check if the ROS2 domain is working
    TEST_F(SimulationInterfaceROS2TestFixture, SmokeTestROS2Domain)
    {
        auto node = GetRos2Node();
        auto pub = node->create_publisher<std_msgs::msg::String>("hello", 20);
        int receivedMsgs = 0;
        auto sub = node->create_subscription<std_msgs::msg::String>(
            "hello",
            10,
            [&receivedMsgs](const std_msgs::msg::String& msg)
            {
                receivedMsgs++;
            });
        std_msgs::msg::String msg;

        msg.data = "Hello World!";
        for (int i = 0; i < 10; ++i)
        {
            pub->publish(msg);
        }
        SpinAppSome();
        EXPECT_EQ(receivedMsgs, 10) << "Did not receive all messages.";
    }

    //! Check if expected services are available and has the default name
    TEST_F(SimulationInterfaceROS2TestFixture, TestIfServicesAvailableInROS2Domain)
    {
        auto node = GetRos2Node();
        auto services = node->get_service_names_and_types();
        ASSERT_FALSE(services.empty()) << "No services available.";

        EXPECT_NE(services.find("/delete_entity"), services.end());
        EXPECT_NE(services.find("/get_entities"), services.end());
        EXPECT_NE(services.find("/get_entities_states"), services.end());
        EXPECT_NE(services.find("/get_entity_state"), services.end());
        EXPECT_NE(services.find("/get_spawnables"), services.end());
        EXPECT_NE(services.find("/set_entity_state"), services.end());
        EXPECT_NE(services.find("/spawn_entity"), services.end());
        EXPECT_NE(services.find("/get_simulator_features"), services.end());
        EXPECT_NE(services.find("/step_simulation"), services.end());
    }

    //! Test if the service call succeeds when the entity is found
    TEST_F(SimulationInterfaceROS2TestFixture, TestDeleteEntity_01)
    {
        using ::testing::_;
        auto node = GetRos2Node();
        auto mock = std::make_shared<SimulationEntityManagerMockedHandler>();
        auto client = node->create_client<simulation_interfaces::srv::DeleteEntity>("/delete_entity");
        auto request = std::make_shared<simulation_interfaces::srv::DeleteEntity::Request>();
        const char TestEntityName[] = "test_entity";
        request->entity = std::string(TestEntityName);
        const AZStd::string entityName = AZStd::string(TestEntityName);

        EXPECT_CALL(*mock, DeleteEntity(entityName, _))
            .WillOnce(::testing::Invoke(
                [](const AZStd::string& name, SimulationInterfaces::DeletionCompletedCb completedCb)
                {
                    EXPECT_EQ(name, "test_entity");
                    completedCb(AZ::Success());
                }));

        auto future = client->async_send_request(request);
        SpinAppUntilFuture(future);

        ASSERT_TRUE(future.wait_for(std::chrono::seconds(0)) == std::future_status::ready) << "Service call timed out.";
        auto response = future.get();
        EXPECT_EQ(response->result.result, simulation_interfaces::msg::Result::RESULT_OK);
    }

    //! Test if the service call fails when the entity is not found
    TEST_F(SimulationInterfaceROS2TestFixture, TestDeleteEntity_02)
    {
        using ::testing::_;
        auto node = GetRos2Node();
        SimulationEntityManagerMockedHandler mock;
        auto client = node->create_client<simulation_interfaces::srv::DeleteEntity>("/delete_entity");
        auto request = std::make_shared<simulation_interfaces::srv::DeleteEntity::Request>();
        const char TestEntityName[] = "test_entity";
        request->entity = std::string(TestEntityName);
        const AZStd::string entityName = AZStd::string(TestEntityName);

        EXPECT_CALL(mock, DeleteEntity(entityName, _))
            .WillOnce(::testing::Invoke(
                [](const AZStd::string& name, SimulationInterfaces::DeletionCompletedCb completedCb)
                {
                    EXPECT_EQ(name, "test_entity");
                    FailedResult failedResult;
                    failedResult.m_errorCode = simulation_interfaces::msg::Result::RESULT_NOT_FOUND,
                    failedResult.m_errorString = "FooBar not found.";
                    completedCb(AZ::Failure(failedResult));
                }));

        auto future = client->async_send_request(request);
        SpinAppUntilFuture(future);

        ASSERT_TRUE(future.wait_for(std::chrono::seconds(0)) == std::future_status::ready) << "Service call timed out.";
        auto response = future.get();
        EXPECT_EQ(response->result.result, simulation_interfaces::msg::Result::RESULT_NOT_FOUND);
    }

    //! Happy path test for GetEntities with a sphere shape
    //! Test if the service is called with a sphere shape and Posix filter is passed through
    TEST_F(SimulationInterfaceROS2TestFixture, GetEntitiesWithShapeSphereAndFilterValid)
    {
        using ::testing::_;
        auto node = GetRos2Node();
        SimulationEntityManagerMockedHandler mock;
        auto client = node->create_client<simulation_interfaces::srv::GetEntities>("/get_entities");
        auto request = std::make_shared<simulation_interfaces::srv::GetEntities::Request>();
        request->filters.bounds.points.resize(2);
        const AZ::Vector3 point1(1.0f, 2.0f, 3.0f);
        request->filters.bounds.points[0].x = point1.GetX();
        request->filters.bounds.points[0].y = point1.GetY();
        request->filters.bounds.points[0].z = point1.GetZ();
        request->filters.bounds.points[1].x = 99.0f;
        request->filters.bounds.type = simulation_interfaces::msg::Bounds::TYPE_SPHERE;
        request->filters.filter = "FooBarFilter";

        EXPECT_CALL(mock, GetEntities(_))
            .WillOnce(::testing::Invoke(
                [=](const EntityFilters& filter)
                {
                    EXPECT_NE(filter.m_boundsShape, nullptr);
                    if (filter.m_boundsShape)
                    {
                        EXPECT_EQ(filter.m_boundsShape->GetShapeType(), Physics::ShapeType::Sphere);
                        Physics::SphereShapeConfiguration* sphereShape =
                            azdynamic_cast<Physics::SphereShapeConfiguration*>(filter.m_boundsShape.get());
                        EXPECT_EQ(sphereShape->m_radius, 99.0f);
                        EXPECT_EQ(sphereShape->m_scale, AZ::Vector3(1.0f));
                    }
                    auto loc = filter.m_boundsPose.GetTranslation();
                    EXPECT_EQ(loc.GetX(), point1.GetX());
                    EXPECT_EQ(loc.GetY(), point1.GetY());
                    EXPECT_EQ(loc.GetZ(), point1.GetZ());

                    EXPECT_EQ(filter.m_nameFilter, "FooBarFilter");
                    return AZ::Success(EntityNameList{ "FooBar" });
                }));

        auto future = client->async_send_request(request);
        SpinAppUntilFuture(future);

        ASSERT_TRUE(future.wait_for(std::chrono::seconds(0)) == std::future_status::ready) << "Service call timed out.";
        auto response = future.get();
        EXPECT_EQ(response->result.result, simulation_interfaces::msg::Result::RESULT_OK);
    }

    //! Try to call the service with invalid data (too few points) for the sphere shape
    TEST_F(SimulationInterfaceROS2TestFixture, GetEntitiesWithShapeSphereWithInvalidData)
    {
        using ::testing::_;
        auto node = GetRos2Node();
        [[maybe_unused]] testing::StrictMock<SimulationEntityManagerMockedHandler> mock;
        auto client = node->create_client<simulation_interfaces::srv::GetEntities>("/get_entities");
        auto request = std::make_shared<simulation_interfaces::srv::GetEntities::Request>();
        request->filters.bounds.points.resize(1); // too few points
        request->filters.bounds.type = simulation_interfaces::msg::Bounds::TYPE_SPHERE;

        auto future = client->async_send_request(request);
        SpinAppUntilFuture(future);
        ASSERT_TRUE(future.wait_for(std::chrono::seconds(0)) == std::future_status::ready) << "Service call timed out.";
        auto response = future.get();
        EXPECT_NE(response->result.result, simulation_interfaces::msg::Result::RESULT_OK);
    }

    //! Happy path test for GetEntities with a box shape
    TEST_F(SimulationInterfaceROS2TestFixture, GetEntitiesWithShapeBox)
    {
        using ::testing::_;
        auto node = GetRos2Node();
        testing::NiceMock<SimulationEntityManagerMockedHandler> mock;
        auto client = node->create_client<simulation_interfaces::srv::GetEntities>("/get_entities");
        auto request = std::make_shared<simulation_interfaces::srv::GetEntities::Request>();
        request->filters.bounds.points.resize(2);

        const AZ::Vector3 point1(1.0f, 2.0f, 3.0f);
        const AZ::Vector3 dims(4.0f, 5.0f, 6.0f);
        const AZ::Vector3 point2 = point1 + dims;

        request->filters.bounds.points[0].x = point1.GetX();
        request->filters.bounds.points[0].y = point1.GetY();
        request->filters.bounds.points[0].z = point1.GetZ();
        request->filters.bounds.points[1].x = point2.GetX();
        request->filters.bounds.points[1].y = point2.GetY();
        request->filters.bounds.points[1].z = point2.GetZ();
        request->filters.bounds.type = simulation_interfaces::msg::Bounds::TYPE_BOX;

        EXPECT_CALL(mock, GetEntities(_))
            .WillOnce(::testing::Invoke(
                [=](const EntityFilters& filter)
                {
                    EXPECT_NE(filter.m_boundsShape, nullptr);
                    if (filter.m_boundsShape)
                    {
                        EXPECT_EQ(filter.m_boundsShape->GetShapeType(), Physics::ShapeType::Box);
                        Physics::BoxShapeConfiguration* boxShape =
                            azdynamic_cast<Physics::BoxShapeConfiguration*>(filter.m_boundsShape.get());
                        EXPECT_EQ(boxShape->m_dimensions.GetX(), dims.GetX());
                        EXPECT_EQ(boxShape->m_dimensions.GetY(), dims.GetY());
                        EXPECT_EQ(boxShape->m_dimensions.GetZ(), dims.GetZ());
                    }
                    auto loc = filter.m_boundsPose.GetTranslation();
                    EXPECT_EQ(loc, AZ::Vector3::CreateZero());
                    return AZ::Success(EntityNameList{ "FooBar" });
                }));

        auto future = client->async_send_request(request);
        SpinAppUntilFuture(future);

        ASSERT_TRUE(future.wait_for(std::chrono::seconds(0)) == std::future_status::ready) << "Service call timed out.";
        auto response = future.get();
        EXPECT_EQ(response->result.result, simulation_interfaces::msg::Result::RESULT_OK);
    }

    //! Try to call the service with invalid data (first point is greater than second point in box)
    TEST_F(SimulationInterfaceROS2TestFixture, GetEntitiesWithShapeBoxInvalid)
    {
        using ::testing::_;
        auto node = GetRos2Node();

        // strict mock, since we don't want to call the real implementation in this test
        [[maybe_unused]] testing::StrictMock<SimulationEntityManagerMockedHandler> mock;

        auto client = node->create_client<simulation_interfaces::srv::GetEntities>("/get_entities");
        auto request = std::make_shared<simulation_interfaces::srv::GetEntities::Request>();
        request->filters.bounds.points.resize(2);

        request->filters.bounds.points[0].x = 2.0f;
        request->filters.bounds.points[0].y = 3.0f;
        request->filters.bounds.points[0].z = 4.0f;
        request->filters.bounds.points[1].x = -2.0f;
        request->filters.bounds.points[1].y = -3.0f;
        request->filters.bounds.points[1].z = 5.0f;
        request->filters.bounds.type = simulation_interfaces::msg::Bounds::TYPE_BOX;

        auto future = client->async_send_request(request);
        SpinAppUntilFuture(future);

        ASSERT_TRUE(future.wait_for(std::chrono::seconds(0)) == std::future_status::ready) << "Service call timed out.";
        auto response = future.get();
        EXPECT_NE(response->result.result, simulation_interfaces::msg::Result::RESULT_OK);
    }

    //! check if capabilities vector is empty when no features are provided by SimulationInterfaces gem
    TEST_F(SimulationInterfaceROS2TestFixture, GetSimulationFeaturesNoFeaturesProvidedByGem)
    {
        using ::testing::_;
        auto node = GetRos2Node();

        auto client = node->create_client<simulation_interfaces::srv::GetSimulatorFeatures>("/get_simulator_features");
        auto request = std::make_shared<simulation_interfaces::srv::GetSimulatorFeatures::Request>();
        auto future = client->async_send_request(request);
        SpinAppUntilFuture(future);
        ASSERT_TRUE(future.wait_for(std::chrono::seconds(0)) == std::future_status::ready) << "Service call timed out.";
        auto response = future.get();
        EXPECT_TRUE(response->features.features.empty()) << "Features vector is not empty.";
    }

    //! check if features are returned when the feature is provided by SimulationFeaturesAggregator
    //! and the feature is not in the list of features provided by the gem
    TEST_F(SimulationInterfaceROS2TestFixture, GetSimulationFeaturesSomeFeaturesProvided)
    {
        SimulationFeaturesAggregatorRequestsMockedHandler mockAggregator;
        using ::testing::_;
        auto node = GetRos2Node();
        AZStd::unordered_set<SimulationFeatureType> features{
            simulation_interfaces::msg::SimulatorFeatures::SPAWNING,
            static_cast<SimulationFeatureType>(0xFE), // invalid feature, should be ignored
            static_cast<SimulationFeatureType>(0xFF), // invalid feature, should be ignored
        };
        EXPECT_CALL(mockAggregator, GetSimulationFeatures())
            .WillOnce(::testing::Invoke(
                [&](void)
                {
                    return features;
                }));

        auto client = node->create_client<simulation_interfaces::srv::GetSimulatorFeatures>("/get_simulator_features");
        auto request = std::make_shared<simulation_interfaces::srv::GetSimulatorFeatures::Request>();
        auto future = client->async_send_request(request);
        SpinAppUntilFuture(future);
        ASSERT_TRUE(future.wait_for(std::chrono::seconds(0)) == std::future_status::ready) << "Service call timed out.";
        auto response = future.get();
        ASSERT_FALSE(response->features.features.empty()) << "Features vector is empty.";
        EXPECT_EQ(response->features.features.size(), 1) << "Features vector should contain one feature only.";
        EXPECT_EQ(response->features.features[0], simulation_interfaces::msg::SimulatorFeatures::SPAWNING) << "Feature is not SPAWNING.";
    }

    //! Check if service succeeds when the name is valid
    TEST_F(SimulationInterfaceROS2TestFixture, TryToSpawnWithValidName)
    {
        using ::testing::_;
        SimulationEntityManagerMockedHandler mock;

        auto node = GetRos2Node();
        auto client = node->create_client<simulation_interfaces::srv::SpawnEntity>("/spawn_entity");
        auto request = std::make_shared<simulation_interfaces::srv::SpawnEntity::Request>();
        request->name = "valid_name";
        request->entity_resource.uri = "test_uri";
        request->entity_namespace = "test_namespace";
        request->allow_renaming = true;

        auto future = client->async_send_request(request);
        EXPECT_CALL(mock, SpawnEntity(_, _, _, _, _, _))
            .WillOnce(::testing::Invoke(
                [](const AZStd::string& name,
                   const AZStd::string& uri,
                   const AZStd::string& entityNamespace,
                   const AZ::Transform& initialPose,
                   const bool allowRename,
                   SpawnCompletedCb completedCb)
                {
                    EXPECT_EQ(name, "valid_name");
                    EXPECT_EQ(uri, "test_uri");
                    EXPECT_EQ(entityNamespace, "test_namespace");
                    EXPECT_TRUE(allowRename);
                    completedCb(AZ::Success("valid_name"));
                }));
        SpinAppUntilFuture(future);

        ASSERT_TRUE(future.wait_for(std::chrono::seconds(0)) == std::future_status::ready) << "Service call timed out.";
        auto response = future.get();
        EXPECT_EQ(response->result.result, simulation_interfaces::msg::Result::RESULT_OK);
    }

    //! Check if service fails when the name is invalid
    TEST_F(SimulationInterfaceROS2TestFixture, TryToSpawnWithInvalidName)
    {
        // strict mock, since we don't want to call the real implementation in this test
        [[maybe_unused]] testing::StrictMock<SimulationEntityManagerMockedHandler> mock;

        auto node = GetRos2Node();
        auto client = node->create_client<simulation_interfaces::srv::SpawnEntity>("/spawn_entity");
        auto request = std::make_shared<simulation_interfaces::srv::SpawnEntity::Request>();
        request->name = "invalid name"; // invalid name
        request->entity_resource.uri = "test_uri";
        request->entity_namespace = "test_namespace";

        auto future = client->async_send_request(request);
        SpinAppUntilFuture(future);

        ASSERT_TRUE(future.wait_for(std::chrono::seconds(0)) == std::future_status::ready) << "Service call timed out.";
        auto response = future.get();
        EXPECT_EQ(response->result.result, simulation_interfaces::srv::SpawnEntity::Response::NAME_INVALID)
            << "Service call should fail with invalid name.";
    }

    //! Check if service fails when the namespace is invalid
    TEST_F(SimulationInterfaceROS2TestFixture, TryToSpawnWithInvalidNamespace)
    {
        // strict mock, since we don't want to call the real implementation in this test
        [[maybe_unused]] testing::StrictMock<SimulationEntityManagerMockedHandler> mock;

        auto node = GetRos2Node();
        auto client = node->create_client<simulation_interfaces::srv::SpawnEntity>("/spawn_entity");
        auto request = std::make_shared<simulation_interfaces::srv::SpawnEntity::Request>();
        request->name = "valid_name";
        request->entity_resource.uri = "test_uri";
        request->entity_namespace = "invalid namespace";

        auto future = client->async_send_request(request);
        SpinAppUntilFuture(future);

        ASSERT_TRUE(future.wait_for(std::chrono::seconds(0)) == std::future_status::ready) << "Service call timed out.";
        auto response = future.get();
        EXPECT_EQ(response->result.result, simulation_interfaces::srv::SpawnEntity::Response::NAMESPACE_INVALID)
            << "Service call should fail with invalid name.";
    }

    TEST_F(SimulationInterfaceROS2TestFixture, SimulationStepsSuccess)
    {
        using ::testing::_;
        auto node = GetRos2Node();
        SimulationManagerMockedHandler mock;
        auto client = rclcpp_action::create_client<simulation_interfaces::action::SimulateSteps>(node, "/simulate_steps");
        auto goal = std::make_shared<simulation_interfaces::action::SimulateSteps::Goal>();
        constexpr AZ::u64 steps = 10;
        goal->steps = steps;

        ASSERT_TRUE(client->wait_for_action_server(std::chrono::seconds(0)) == true) << "Action server is unavailable";

        EXPECT_CALL(mock, IsSimulationPaused())
            .WillOnce(::testing::Invoke(
                []()
                {
                    return true;
                }));

        EXPECT_CALL(mock, StepSimulation(steps));

        EXPECT_CALL(mock, IsSimulationStepsActive())
            .WillOnce(::testing::Invoke(
                []()
                {
                    return false;
                }));

        auto future = client->async_send_goal(*goal);
        SpinAppUntilFuture(future);
        ASSERT_TRUE(future.wait_for(std::chrono::seconds(0)) == std::future_status::ready) << "Action call timed out.";
        auto goalHandle = future.get();
        ASSERT_NE(goalHandle, nullptr);
        auto result = client->async_get_result(goalHandle);
        SpinAppUntilFuture(result);
        ASSERT_TRUE(result.wait_for(std::chrono::seconds(0)) == std::future_status::ready) << "Action call timed out.";
        EXPECT_EQ(result.get().result->result.result, simulation_interfaces::msg::Result::RESULT_OK);
    }

    TEST_F(SimulationInterfaceROS2TestFixture, SimulationStepsFailureSimulationIsNotPaused)
    {
        using ::testing::_;
        auto node = GetRos2Node();
        SimulationManagerMockedHandler mock;
        auto client = rclcpp_action::create_client<simulation_interfaces::action::SimulateSteps>(node, "/simulate_steps");
        auto goal = std::make_shared<simulation_interfaces::action::SimulateSteps::Goal>();
        goal->steps = 10;

        ASSERT_TRUE(client->wait_for_action_server(std::chrono::seconds(0)) == true) << "Action server is unavailable";

        EXPECT_CALL(mock, IsSimulationPaused())
            .WillOnce(::testing::Invoke(
                []()
                {
                    return false;
                }));

        auto future = client->async_send_goal(*goal);
        SpinAppUntilFuture(future);
        ASSERT_TRUE(future.wait_for(std::chrono::seconds(0)) == std::future_status::ready) << "Action call timed out.";
        auto goalHandle = future.get();
        ASSERT_NE(goalHandle, nullptr);
        auto result = client->async_get_result(goalHandle);
        SpinAppUntilFuture(result);
        ASSERT_TRUE(result.wait_for(std::chrono::seconds(0)) == std::future_status::ready) << "Action call timed out.";
        EXPECT_EQ(result.get().result->result.result, simulation_interfaces::msg::Result::RESULT_OPERATION_FAILED);
    }

    TEST_F(SimulationInterfaceROS2TestFixture, SimulationStepsCancelled)
    {
        using ::testing::_;
        auto node = GetRos2Node();
        SimulationManagerMockedHandler mock;
        auto client = rclcpp_action::create_client<simulation_interfaces::action::SimulateSteps>(node, "/simulate_steps");
        auto goal = std::make_shared<simulation_interfaces::action::SimulateSteps::Goal>();
        constexpr AZ::u64 steps = 10;
        goal->steps = steps;

        ASSERT_TRUE(client->wait_for_action_server(std::chrono::seconds(0)) == true) << "Action server is unavailable";

        EXPECT_CALL(mock, IsSimulationPaused())
            .WillOnce(::testing::Invoke(
                []()
                {
                    return true;
                }));

        EXPECT_CALL(mock, StepSimulation(steps));

        EXPECT_CALL(mock, IsSimulationStepsActive())
            .WillRepeatedly(::testing::Invoke(
                []()
                {
                    return true;
                }));

        EXPECT_CALL(mock, CancelStepSimulation());

        auto future = client->async_send_goal(*goal);
        SpinAppUntilFuture(future);
        ASSERT_TRUE(future.wait_for(std::chrono::seconds(0)) == std::future_status::ready) << "Action call timed out.";
        auto goalHandle = future.get();
        ASSERT_NE(goalHandle, nullptr);
        auto cancelFuture = client->async_cancel_goal(goalHandle);
        SpinAppUntilFuture(cancelFuture);
        ASSERT_TRUE(cancelFuture.wait_for(std::chrono::seconds(0)) == std::future_status::ready) << "Action call timed out.";
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
