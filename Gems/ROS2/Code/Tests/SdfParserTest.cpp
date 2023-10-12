/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <AzCore/UnitTest/TestTypes.h>
#include <AzCore/std/smart_ptr/make_shared.h>
#include <AzTest/AzTest.h>
#include <AzTest/Utils.h>
#include <RobotImporter/SDFormat/ROS2SensorHooks.h>
#include <RobotImporter/Utils/ErrorUtils.h>
#include <RobotImporter/Utils/RobotImporterUtils.h>
#include <RobotImporter/URDF/UrdfParser.h>

namespace UnitTest
{
    class SdfParserTest
      : public LeakDetectionFixture
    {
    public:
        std::string GetSdfWithTwoSensors()
        {
            return R"(<?xml version="1.0"?>
                      <sdf version="1.6">
                        <model name="test_two_sensors">
                          <link name="link1">
                            <sensor name="camera" type="camera">
                              <pose>0 0 0 0 0 0</pose>
                              <camera>
                                <horizontal_fov>2.0</horizontal_fov>
                                <image>
                                  <width>640</width>
                                  <height>480</height>
                                </image>
                                <clip>
                                  <near>0.01</near>
                                  <far>1000</far>
                                </clip>
                              </camera>
                              <update_rate>10</update_rate>
                              <plugin name="camera_plug" filename="libgazebo_ros_camera.so">
                                <camera_name>custom_camera</camera_name>
                              </plugin>
                            </sensor>
                          </link>
                          <link name="link2">
                            <sensor name="laser" type="ray">
                              <always_on>1</always_on>
                              <visualize>1</visualize>
                              <update_rate>20.0</update_rate>
                              <pose>0 0 0 0 0 0</pose>
                              <ray>
                                <scan>
                                  <horizontal>
                                    <samples>640</samples>
                                    <resolution>1.0</resolution>
                                    <min_angle>-2.0</min_angle>
                                    <max_angle>2.5</max_angle>
                                  </horizontal>
                                </scan>
                                <range>
                                  <min>0.02</min>
                                  <max>10</max>
                                  <resolution>0.01</resolution>
                                </range>
                              </ray>
                              <plugin name="laser_plug" filename="librayplugin.so"/>
                            </sensor>
                          </link>
                        </model>
                      </sdf>)";
        }

        static std::string GetSdfWithImuSensor()
        {
            return R"(<?xml version="1.0"?>
                      <sdf version="1.6">
                        <model name="test_imu_sensor">
                          <link name="link1">
                            <sensor name="link1_imu" type="imu">
                              <always_on>true</always_on>
                              <update_rate>200</update_rate>
                              <imu>
                                <angular_velocity>
                                  <x>
                                    <noise type="gaussian">
                                      <mean>0.0</mean>
                                      <stddev>2e-4</stddev>
                                    </noise>
                                  </x>
                                  <y>
                                    <noise type="gaussian">
                                      <mean>0.0</mean>
                                      <stddev>3e-4</stddev>
                                    </noise>
                                  </y>
                                  <z>
                                    <noise type="gaussian">
                                      <mean>0.0</mean>
                                      <stddev>4e-4</stddev>
                                    </noise>
                                  </z>
                                </angular_velocity>
                                <linear_acceleration>
                                  <x>
                                    <noise type="gaussian">
                                      <mean>0.0</mean>
                                      <stddev>1.7e-2</stddev>
                                    </noise>
                                  </x>
                                  <y>
                                    <noise type="gaussian">
                                      <mean>0.0</mean>
                                      <stddev>1.8e-2</stddev>
                                    </noise>
                                  </y>
                                  <z>
                                    <noise type="gaussian">
                                      <mean>0.0</mean>
                                      <stddev>1.9e-2</stddev>
                                    </noise>
                                  </z>
                                </linear_acceleration>
                              </imu>
                              <plugin name="imu_plugin" filename="libgazebo_ros_imu_sensor.so">
                                <ros>
                                  <argument>~/out:=imu</argument>
                                </ros>
                              </plugin>
                            </sensor>
                          </link>
                        </model>
                      </sdf>)";
        }

        static std::string GetSdfWithDuplicateModelName()
        {
            return R"(<?xml version="1.0"?>
            <sdf version="1.7">
            <model name="root_model">
              <link name="root_link"/>
            </model>
            <world name="default">
              <model name="my_model">
                <link name="link1"/>
              </model>
              <model name="your_model">
                <link name="link2"/>
              </model>
              <model name="my_model">
                <link name="link3"/>
              </model>
            </world>
          </sdf>)";
        }

        static std::string GetSdfWithWorldThatHasMultipleModels()
        {
            return R"(<?xml version="1.0"?>
            <sdf version="1.8">
            <model name="root_model">
              <link name="root_link"/>
            </model>
            <world name="default">
              <model name="my_model">
                <link name="link1"/>
              </model>
              <model name="your_model">
                <link name="link2"/>
              </model>
            </world>
            <world name="second">
              <model name="every_model">
                <link name="link3"/>
              </model>
            </world>
          </sdf>)";
        }

        static std::string GetSdfWithMultipleModelsThatHaveLinksWithTheSameName()
        {
            return R"(<?xml version="1.0"?>
            <sdf version="1.10">
            <world name="default">
              <model name="my_model">
                <link name="same_link_name"/>
              </model>
              <model name="your_model">
                <link name="same_link_name"/>
              </model>
            </world>
          </sdf>)";
        }

        struct SdfXmlStack
        {
            AZStd::deque<std::string> m_stack;
        };
        static SdfXmlStack GetSdfWorldWithNestedModelWithPose()
        {
            SdfXmlStack sdfXmlStack;
            sdfXmlStack.m_stack.emplace_back(R"(<?xml version="1.0"?>
            <sdf version="1.10">
                <world name="default">
                    <model name="top_model">
                        <include>
                            <uri>model://nested_test.sdf</uri>
                        </include>
                        <pose>8 2 4 0 -0 -1.564217</pose>
                    </model>
                </world>
            </sdf>)");

            sdfXmlStack.m_stack.emplace_back(R"(<?xml version="1.0"?>
            <sdf version="1.10">
                <model name="nested_model">
                    <link name="link">
                        <inertial>
                            <mass>50</mass>
                        </inertial>
                    </link>
                </model>
            </sdf>)");

            return sdfXmlStack;
        }
    };

    TEST_F(SdfParserTest, SdfWithDuplicateModelNames_ResultsInError)
    {
        const auto xmlStr = GetSdfWithDuplicateModelName();
        const auto sdfRootOutcome = ROS2::UrdfParser::Parse(xmlStr, {});
        ASSERT_FALSE(sdfRootOutcome);
        const auto& sdfErrors = sdfRootOutcome.GetSdfErrors();
        EXPECT_FALSE(sdfErrors.empty());
        AZStd::string errorString = ROS2::Utils::JoinSdfErrorsToString(sdfRootOutcome.GetSdfErrors());
        printf("SDF with duplicate model names failed to parse with errors: %s\n", errorString.c_str());
    }

    TEST_F(SdfParserTest, SdfWithModelsOnRootAndWorld_ParsesSuccessfully)
    {
        const auto xmlStr = GetSdfWithWorldThatHasMultipleModels();
        const auto sdfRootOutcome = ROS2::UrdfParser::Parse(xmlStr, {});
        ASSERT_TRUE(sdfRootOutcome);
        const auto& sdfRoot = sdfRootOutcome.GetRoot();
        // This SDF should have a model on the root that points to root model
        const auto* rootSdfModel = sdfRoot.Model();
        ASSERT_NE(nullptr, rootSdfModel);
        EXPECT_EQ("root_model", rootSdfModel->Name());
        EXPECT_TRUE(rootSdfModel->LinkNameExists("root_link"));

        // The SDF should also have a world on the root as well
        ASSERT_EQ(2, sdfRoot.WorldCount());
        const auto* sdfWorld = sdfRoot.WorldByIndex(0);
        ASSERT_NE(nullptr, sdfWorld);

        // Also validate that the world can be looked up by name
        sdfWorld = sdfRoot.WorldByName("default");
        ASSERT_NE(nullptr, sdfWorld);

        EXPECT_EQ(2, sdfWorld->ModelCount());

        const auto* myModel = sdfWorld->ModelByName("my_model");
        ASSERT_NE(nullptr, myModel);
        EXPECT_TRUE(myModel->LinkNameExists("link1"));

        const auto* yourModel = sdfWorld->ModelByName("your_model");
        ASSERT_NE(nullptr, yourModel);
        EXPECT_TRUE(yourModel->LinkNameExists("link2"));

        sdfWorld = sdfRoot.WorldByName("second");
        ASSERT_NE(nullptr, sdfWorld);

        const auto* everyModel = sdfWorld->ModelByName("every_model");
        ASSERT_NE(nullptr, everyModel);
        EXPECT_TRUE(everyModel->LinkNameExists("link3"));

        AZStd::vector<const sdf::Model*> models;
        // Test visitation return results. All model siblings and nested models are visited
        auto StoreModelAndVisitNestedModelsAndSiblings =
            [&models](const sdf::Model& model, const ROS2::Utils::ModelStack&) -> ROS2::Utils::VisitModelResponse
        {
            models.push_back(&model);
            return ROS2::Utils::VisitModelResponse::VisitNestedAndSiblings;
        };
        ROS2::Utils::VisitModels(sdfRoot, StoreModelAndVisitNestedModelsAndSiblings);

        ASSERT_EQ(4, models.size());
        EXPECT_EQ("root_model", models[0]->Name());
        EXPECT_EQ("my_model", models[1]->Name());
        EXPECT_EQ("your_model", models[2]->Name());
        EXPECT_EQ("every_model", models[3]->Name());

        // Test visiting models where siblings visited
        // In this case only models directly on the SDF root object
        // or directory child of the sdf world has there models visited
        models.clear();
        auto StoreModelAndVisitSiblings =
            [&models](const sdf::Model& model, const ROS2::Utils::ModelStack&) -> ROS2::Utils::VisitModelResponse
        {
            models.push_back(&model);
            return ROS2::Utils::VisitModelResponse::VisitSiblings;
        };
        ROS2::Utils::VisitModels(sdfRoot, StoreModelAndVisitSiblings);

        ASSERT_EQ(4, models.size());
        EXPECT_EQ("root_model", models[0]->Name());
        EXPECT_EQ("my_model", models[1]->Name());
        EXPECT_EQ("your_model", models[2]->Name());
        EXPECT_EQ("every_model", models[3]->Name());

        // Visit only the first model and stop any futher visitation
        models.clear();
        auto StoreModelAndStop = [&models](const sdf::Model& model, const ROS2::Utils::ModelStack&) -> ROS2::Utils::VisitModelResponse
        {
            models.push_back(&model);
            return ROS2::Utils::VisitModelResponse::Stop;
        };
        ROS2::Utils::VisitModels(sdfRoot, StoreModelAndStop);

        ASSERT_EQ(1, models.size());
        EXPECT_EQ("root_model", models[0]->Name());
    }

    TEST_F(SdfParserTest, VisitingSdfWithMultipleModelsWithSameLinkName_VisitsAllLinks)
    {
        const auto xmlStr = GetSdfWithMultipleModelsThatHaveLinksWithTheSameName();
        const auto sdfRootOutcome = ROS2::UrdfParser::Parse(xmlStr, {});
        ASSERT_TRUE(sdfRootOutcome);
        const auto& sdfRoot = sdfRootOutcome.GetRoot();
        // The SDF should also have a single world
        ASSERT_EQ(1, sdfRoot.WorldCount());
        const auto* sdfWorld = sdfRoot.WorldByIndex(0);
        ASSERT_NE(nullptr, sdfWorld);

        // There should be two models of "my_model" and "your_model"
        EXPECT_EQ(2, sdfWorld->ModelCount());

        const auto* myModel = sdfWorld->ModelByName("my_model");
        ASSERT_NE(nullptr, myModel);
        EXPECT_TRUE(myModel->LinkNameExists("same_link_name"));

        const auto* yourModel = sdfWorld->ModelByName("your_model");
        ASSERT_NE(nullptr, yourModel);
        EXPECT_TRUE(yourModel->LinkNameExists("same_link_name"));

        // Make sure that all links are gathered
        AZStd::unordered_map<AZStd::string, const sdf::Link*> links = ROS2::Utils::GetAllLinks(*myModel, true);
        auto otherLinks = ROS2::Utils::GetAllLinks(*yourModel, true);
        links.insert(AZStd::move(otherLinks.begin()), AZStd::move(otherLinks.end()));

        ASSERT_EQ(2, links.size());
        EXPECT_TRUE(links.contains("my_model::same_link_name"));
        EXPECT_TRUE(links.contains("your_model::same_link_name"));
    }

    TEST_F(SdfParserTest, NestedModel_CanBeIncludedFromURI_Succeeds)
    {
        const SdfXmlStack sdfStack = GetSdfWorldWithNestedModelWithPose();
        ASSERT_EQ(2, sdfStack.m_stack.size());

        // First create the model file in a temporary directory and setup
        // the model URI prefixes
        const std::string& nestedModelXml = sdfStack.m_stack.back();

        constexpr AZ::IO::PathView nestedModelFilePath = "nested_test.sdf";
        AZ::Test::ScopedAutoTempDirectory tempDirectory;
        AZStd::optional<AZ::IO::FixedMaxPath> nestedModelFullPath =
            AZ::Test::CreateTestFile(tempDirectory, nestedModelFilePath, AZStd::string_view(nestedModelXml.data(), nestedModelXml.size()));

        // Verify the nested test SDF file has been created
        ASSERT_TRUE(nestedModelFullPath.has_value());

        // Now grab the world sdf content and use the Parse command to parse the contents
        const auto xmlStr = sdfStack.m_stack.front();

        sdf::ParserConfig sdfConfig;
        // Add the temporary directory as a URI prefix
        sdfConfig.AddURIPath("model://", tempDirectory.GetDirectory());

        auto SdfFindCallback = [](const std::string& fileName) -> std::string
        {
            ADD_FAILURE() << "File " << fileName << " was not found in UnitTest.\n";
            return std::string{};
        };

        sdfConfig.SetFindCallback(AZStd::move(SdfFindCallback));

        const auto sdfRootOutcome = ROS2::UrdfParser::Parse(xmlStr, sdfConfig);
        ASSERT_TRUE(sdfRootOutcome);
        const auto& sdfRoot = sdfRootOutcome.GetRoot();
        // The SDF should also have a single world
        ASSERT_EQ(1, sdfRoot.WorldCount());
        const auto* sdfWorld = sdfRoot.WorldByIndex(0);
        ASSERT_NE(nullptr, sdfWorld);

        // There should be only one model on the world that is the "top_model"
        EXPECT_EQ(1, sdfWorld->ModelCount());

        // The nested model contains the name from within
        const auto* topModel = sdfWorld->ModelByName("top_model");
        ASSERT_NE(nullptr, topModel);

        gz::math::Pose3d modelPose = topModel->RawPose();
        EXPECT_DOUBLE_EQ(8.0, modelPose.X());
        EXPECT_DOUBLE_EQ(2.0, modelPose.Y());
        EXPECT_DOUBLE_EQ(4.0, modelPose.Z());
        EXPECT_DOUBLE_EQ(-1.564217, modelPose.Yaw());

        ASSERT_EQ(1, topModel->ModelCount());
        const auto* nestedModel = topModel->ModelByName("nested_model");
        ASSERT_NE(nullptr, nestedModel);

        // The nested model should have a link on it
        EXPECT_TRUE(nestedModel->LinkNameExists("link"));
        // The link name can also be looked up using the relative scoped name as well from the parent top model
        EXPECT_TRUE(topModel->LinkNameExists("nested_model::link"));
    }

    TEST_F(SdfParserTest, CheckModelCorrectness)
    {
        {
            const auto xmlStr = GetSdfWithTwoSensors();
            const auto sdfRootOutcome = ROS2::UrdfParser::Parse(xmlStr, {});
            ASSERT_TRUE(sdfRootOutcome);
            const auto& sdfRoot = sdfRootOutcome.GetRoot();
            const auto* sdfModel = sdfRoot.Model();
            ASSERT_NE(nullptr, sdfModel);

            EXPECT_EQ(sdfModel->Name(), "test_two_sensors");
            EXPECT_EQ(sdfModel->LinkCount(), 2U);

            const auto* link1 = sdfModel->LinkByName("link1");
            ASSERT_TRUE(link1);
            EXPECT_EQ(link1->SensorCount(), 1U);
            const auto* sensor1 = link1->SensorByIndex(0U);
            ASSERT_TRUE(sensor1);
            EXPECT_EQ(sensor1->Type(), sdf::SensorType::CAMERA);
            EXPECT_EQ(sensor1->UpdateRate(), 10);
            auto* cameraSensor = sensor1->CameraSensor();
            ASSERT_TRUE(cameraSensor);
            EXPECT_EQ(cameraSensor->ImageWidth(), 640);
            EXPECT_EQ(cameraSensor->ImageHeight(), 480);
            EXPECT_NEAR(cameraSensor->HorizontalFov().Radian(), 2.0, 1e-5);
            EXPECT_NEAR(cameraSensor->NearClip(), 0.01, 1e-5);
            EXPECT_NEAR(cameraSensor->FarClip(), 1000, 1e-5);
            EXPECT_EQ(sensor1->Plugins().size(), 1U);
            EXPECT_EQ(sensor1->Plugins().at(0).Name(), "camera_plug");
            EXPECT_EQ(sensor1->Plugins().at(0).Filename(), "libgazebo_ros_camera.so");

            const auto* link2 = sdfModel->LinkByName("link2");
            ASSERT_TRUE(link2);
            EXPECT_EQ(link2->SensorCount(), 1U);
            const auto* sensor2 = link2->SensorByIndex(0U);
            ASSERT_TRUE(sensor2);
            EXPECT_EQ(sensor2->Type(), sdf::SensorType::LIDAR);
            EXPECT_EQ(sensor2->UpdateRate(), 20);
            auto* lidarSensor = sensor2->LidarSensor();
            ASSERT_TRUE(lidarSensor);
            EXPECT_EQ(lidarSensor->HorizontalScanSamples(), 640);
            EXPECT_NEAR(lidarSensor->HorizontalScanResolution(), 1.0, 1e-5);
            EXPECT_NEAR(lidarSensor->HorizontalScanMinAngle().Radian(), -2.0, 1e-5);
            EXPECT_NEAR(lidarSensor->HorizontalScanMaxAngle().Radian(), 2.5, 1e-5);
            EXPECT_NEAR(lidarSensor->RangeResolution(), 0.01, 1e-5);
            EXPECT_NEAR(lidarSensor->RangeMin(), 0.02, 1e-5);
            EXPECT_NEAR(lidarSensor->RangeMax(), 10.0, 1e-5);
            EXPECT_EQ(sensor2->Plugins().size(), 1U);
            EXPECT_EQ(sensor2->Plugins().at(0).Name(), "laser_plug");
            EXPECT_EQ(sensor2->Plugins().at(0).Filename(), "librayplugin.so");
        }

        {
            const auto xmlStr = GetSdfWithImuSensor();
            const auto sdfRootOutcome = ROS2::UrdfParser::Parse(xmlStr, {});
            ASSERT_TRUE(sdfRootOutcome);
            const auto& sdfRoot = sdfRootOutcome.GetRoot();
            const auto* sdfModel = sdfRoot.Model();
            ASSERT_NE(nullptr, sdfModel);

            EXPECT_EQ(sdfModel->Name(), "test_imu_sensor");
            EXPECT_EQ(sdfModel->LinkCount(), 1U);

            const auto* link1 = sdfModel->LinkByName("link1");
            ASSERT_TRUE(link1);
            EXPECT_EQ(link1->SensorCount(), 1U);
            const auto* sensor1 = link1->SensorByIndex(0U);
            ASSERT_TRUE(sensor1);
            EXPECT_EQ(sensor1->Type(), sdf::SensorType::IMU);
            EXPECT_EQ(sensor1->Name(), "link1_imu");
            EXPECT_EQ(sensor1->UpdateRate(), 200);

            auto* imuSensor = sensor1->ImuSensor();
            ASSERT_TRUE(imuSensor);
            EXPECT_EQ(imuSensor->AngularVelocityXNoise().Type(), sdf::NoiseType::GAUSSIAN);
            EXPECT_EQ(imuSensor->AngularVelocityYNoise().Type(), sdf::NoiseType::GAUSSIAN);
            EXPECT_EQ(imuSensor->AngularVelocityZNoise().Type(), sdf::NoiseType::GAUSSIAN);
            EXPECT_EQ(imuSensor->AngularVelocityXNoise().Mean(), 0.0);
            EXPECT_EQ(imuSensor->AngularVelocityYNoise().Mean(), 0.0);
            EXPECT_EQ(imuSensor->AngularVelocityZNoise().Mean(), 0.0);
            EXPECT_NEAR(imuSensor->AngularVelocityXNoise().StdDev(), 2e-4, 1e-5);
            EXPECT_NEAR(imuSensor->AngularVelocityYNoise().StdDev(), 3e-4, 1e-5);
            EXPECT_NEAR(imuSensor->AngularVelocityZNoise().StdDev(), 4e-4, 1e-5);
            EXPECT_EQ(imuSensor->LinearAccelerationXNoise().Type(), sdf::NoiseType::GAUSSIAN);
            EXPECT_EQ(imuSensor->LinearAccelerationYNoise().Type(), sdf::NoiseType::GAUSSIAN);
            EXPECT_EQ(imuSensor->LinearAccelerationZNoise().Type(), sdf::NoiseType::GAUSSIAN);
            EXPECT_EQ(imuSensor->LinearAccelerationXNoise().Mean(), 0.0);
            EXPECT_EQ(imuSensor->LinearAccelerationYNoise().Mean(), 0.0);
            EXPECT_EQ(imuSensor->LinearAccelerationZNoise().Mean(), 0.0);
            EXPECT_NEAR(imuSensor->LinearAccelerationXNoise().StdDev(), 1.7e-2, 1e-5);
            EXPECT_NEAR(imuSensor->LinearAccelerationYNoise().StdDev(), 1.8e-2, 1e-5);
            EXPECT_NEAR(imuSensor->LinearAccelerationZNoise().StdDev(), 1.9e-2, 1e-5);

            EXPECT_EQ(sensor1->Plugins().size(), 1U);
            EXPECT_EQ(sensor1->Plugins().at(0).Name(), "imu_plugin");
            EXPECT_EQ(sensor1->Plugins().at(0).Filename(), "libgazebo_ros_imu_sensor.so");
        }
    }

    TEST_F(SdfParserTest, RobotImporterUtils)
    {
        AZStd::unordered_set<AZStd::string> supportedPlugins{ "libgazebo_ros_camera.so", "libgazebo_ros_openni_kinect.so" };
        sdf::Plugin plug;

        plug.SetName("test_camera");
        plug.SetFilename("libgazebo_ros_camera.so");
        EXPECT_EQ("libgazebo_ros_camera.so", ROS2::Utils::SDFormat::GetPluginFilename(plug));
        EXPECT_TRUE(ROS2::Utils::SDFormat::IsPluginSupported(plug, supportedPlugins));
        plug.SetFilename("/usr/lib/libgazebo_ros_camera.so");
        EXPECT_EQ("libgazebo_ros_camera.so", ROS2::Utils::SDFormat::GetPluginFilename(plug));
        EXPECT_TRUE(ROS2::Utils::SDFormat::IsPluginSupported(plug, supportedPlugins));
        plug.SetFilename("~/dev/libgazebo_ros_camera.so");
        EXPECT_EQ("libgazebo_ros_camera.so", ROS2::Utils::SDFormat::GetPluginFilename(plug));
        EXPECT_TRUE(ROS2::Utils::SDFormat::IsPluginSupported(plug, supportedPlugins));
        plug.SetFilename("fun.so");
        EXPECT_EQ("fun.so", ROS2::Utils::SDFormat::GetPluginFilename(plug));
        EXPECT_FALSE(ROS2::Utils::SDFormat::IsPluginSupported(plug, supportedPlugins));
        plug.SetFilename("fun");
        EXPECT_EQ("fun", ROS2::Utils::SDFormat::GetPluginFilename(plug));
        EXPECT_FALSE(ROS2::Utils::SDFormat::IsPluginSupported(plug, supportedPlugins));

        AZStd::unordered_set<AZStd::string> cameraSupportedParams{
            ">update_rate", ">camera>horizontal_fov", ">camera>image>width", ">camera>image>height"
        };

        const auto xmlStr = GetSdfWithTwoSensors();
        const auto sdfRootOutcome = ROS2::UrdfParser::Parse(xmlStr, {});
        ASSERT_TRUE(sdfRootOutcome);
        const auto& sdfRoot = sdfRootOutcome.GetRoot();
        const auto* sdfModel = sdfRoot.Model();
        ASSERT_NE(nullptr, sdfModel);
        const sdf::ElementPtr cameraElement = sdfModel->LinkByName("link1")->SensorByIndex(0U)->Element();
        const sdf::ElementPtr laserElement = sdfModel->LinkByName("link2")->SensorByIndex(0U)->Element();

        {
            const auto& unsupportedCameraParams = ROS2::Utils::SDFormat::GetUnsupportedParams(cameraElement, cameraSupportedParams);
            EXPECT_EQ(unsupportedCameraParams.size(), 3U);
            EXPECT_EQ(unsupportedCameraParams[0U], ">pose");
            EXPECT_EQ(unsupportedCameraParams[1U], ">camera>clip>near");
            EXPECT_EQ(unsupportedCameraParams[2U], ">camera>clip>far");
        }

        cameraSupportedParams.emplace(">pose");
        {
            const auto& unsupportedCameraParams = ROS2::Utils::SDFormat::GetUnsupportedParams(cameraElement, cameraSupportedParams);
            EXPECT_EQ(unsupportedCameraParams.size(), 2U);
            EXPECT_EQ(unsupportedCameraParams[0U], ">camera>clip>near");
            EXPECT_EQ(unsupportedCameraParams[1U], ">camera>clip>far");
        }

        cameraSupportedParams.emplace(">camera>clip>near");
        cameraSupportedParams.emplace(">camera>clip>far");
        {
            const auto& unsupportedCameraParams = ROS2::Utils::SDFormat::GetUnsupportedParams(cameraElement, cameraSupportedParams);
            EXPECT_EQ(unsupportedCameraParams.size(), 0U);
        }

        const AZStd::unordered_set<AZStd::string> laserSupportedParams{ ">pose",
                                                                        ">update_rate",
                                                                        ">ray>scan>horizontal>samples",
                                                                        ">ray>scan>horizontal>resolution",
                                                                        ">ray>scan>horizontal>min_angle",
                                                                        ">ray>scan>horizontal>max_angle",
                                                                        ">ray>range>min",
                                                                        ">ray>range>max",
                                                                        ">ray>range>resolution",
                                                                        ">always_on",
                                                                        ">visualize" };
        const auto& unsupportedLaserParams = ROS2::Utils::SDFormat::GetUnsupportedParams(laserElement, laserSupportedParams);
        EXPECT_EQ(unsupportedLaserParams.size(), 0U);
    }

    TEST_F(SdfParserTest, SensorPluginImporterHookCheck)
    {
        {
            const auto xmlStr = GetSdfWithTwoSensors();
            const auto sdfRootOutcome = ROS2::UrdfParser::Parse(xmlStr, {});
            ASSERT_TRUE(sdfRootOutcome);
            const auto& sdfRoot = sdfRootOutcome.GetRoot();
            const auto* sdfModel = sdfRoot.Model();
            ASSERT_NE(nullptr, sdfModel);
            const sdf::ElementPtr cameraElement = sdfModel->LinkByName("link1")->SensorByIndex(0U)->Element();
            const auto& cameraImporterHook = ROS2::SDFormat::ROS2SensorHooks::ROS2CameraSensor();

            const auto& unsupportedCameraParams =
                ROS2::Utils::SDFormat::GetUnsupportedParams(cameraElement, cameraImporterHook.m_supportedSensorParams);
            EXPECT_EQ(unsupportedCameraParams.size(), 1U);
            EXPECT_EQ(unsupportedCameraParams[0U], ">pose");

            sdf::Plugin plug;
            plug.SetName("test_camera");
            plug.SetFilename("libgazebo_ros_camera.so");
            EXPECT_TRUE(ROS2::Utils::SDFormat::IsPluginSupported(plug, cameraImporterHook.m_pluginNames));
            plug.SetFilename("/usr/lib/libgazebo_ros_openni_kinect.so");
            EXPECT_TRUE(ROS2::Utils::SDFormat::IsPluginSupported(plug, cameraImporterHook.m_pluginNames));
            plug.SetFilename("~/dev/libgazebo_ros_imu.so");
            EXPECT_FALSE(ROS2::Utils::SDFormat::IsPluginSupported(plug, cameraImporterHook.m_pluginNames));
            plug.SetFilename("libgazebo_ros_camera");
            EXPECT_FALSE(ROS2::Utils::SDFormat::IsPluginSupported(plug, cameraImporterHook.m_pluginNames));

            EXPECT_TRUE(cameraImporterHook.m_sensorTypes.contains(sdf::SensorType::CAMERA));
            EXPECT_TRUE(cameraImporterHook.m_sensorTypes.contains(sdf::SensorType::DEPTH_CAMERA));
            EXPECT_FALSE(cameraImporterHook.m_sensorTypes.contains(sdf::SensorType::GPS));

            const sdf::ElementPtr lidarElement = sdfModel->LinkByName("link2")->SensorByIndex(0U)->Element();
            const auto& lidarImporterHook = ROS2::SDFormat::ROS2SensorHooks::ROS2LidarSensor();
            const auto& unsupportedLidarParams =
                ROS2::Utils::SDFormat::GetUnsupportedParams(lidarElement, lidarImporterHook.m_supportedSensorParams);
            EXPECT_EQ(unsupportedLidarParams.size(), 5U);
            EXPECT_EQ(unsupportedLidarParams[0U], ">always_on");
            EXPECT_EQ(unsupportedLidarParams[1U], ">visualize");
            EXPECT_EQ(unsupportedLidarParams[2U], ">pose");
            EXPECT_EQ(unsupportedLidarParams[3U], ">ray>scan>horizontal>resolution");
            EXPECT_EQ(unsupportedLidarParams[4U], ">ray>range>resolution");
        }
        {
            const auto xmlStr = GetSdfWithImuSensor();
            const auto sdfRootOutcome = ROS2::UrdfParser::Parse(xmlStr, {});
            ASSERT_TRUE(sdfRootOutcome);
            const auto& sdfRoot = sdfRootOutcome.GetRoot();
            const auto* sdfModel = sdfRoot.Model();
            ASSERT_NE(nullptr, sdfModel);
            const sdf::ElementPtr imuElement = sdfModel->LinkByName("link1")->SensorByIndex(0U)->Element();
            const auto& importerHook = ROS2::SDFormat::ROS2SensorHooks::ROS2ImuSensor();

            const auto& unsupportedImuParams =
                ROS2::Utils::SDFormat::GetUnsupportedParams(imuElement, importerHook.m_supportedSensorParams);
            EXPECT_EQ(unsupportedImuParams.size(), 1U);
            EXPECT_EQ(unsupportedImuParams[0U], ">always_on");
        }
    }

} // namespace UnitTest
