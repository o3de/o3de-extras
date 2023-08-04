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
#include <RobotImporter/SDFormat/ROS2SensorHooks.h>
#include <RobotImporter/Utils/RobotImporterUtils.h>

#include <sdf/sdf.hh>

namespace UnitTest
{

    class SdfParserTest : public LeakDetectionFixture
    {
    public:
        AZStd::shared_ptr<sdf::Root> Parse(const std::string& xmlString)
        {
            sdf::SDFPtr sdfElement(new sdf::SDF());
            sdf::init(sdfElement);
            sdf::Errors readErrors;
            const bool success = sdf::readString(xmlString, sdfElement, readErrors);
            if (!success)
            {
                return nullptr;
            }

            auto sdfDom = AZStd::make_shared<sdf::Root>();
            sdf::Errors parseErrors = sdfDom->Load(sdfElement);
            return sdfDom;
        }

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
    };

    TEST_F(SdfParserTest, CheckModelCorrectness)
    {
        const auto xmlStr = GetSdfWithTwoSensors();
        const auto sdfRoot = Parse(xmlStr);
        ASSERT_TRUE(sdfRoot);
        const auto* sdfModel = sdfRoot->Model();
        ASSERT_TRUE(sdfModel);

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
        const auto sdfRoot = Parse(xmlStr);
        const auto* sdfModel = sdfRoot->Model();
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
        const auto xmlStr = GetSdfWithTwoSensors();
        const auto sdfRoot = Parse(xmlStr);
        const auto* sdfModel = sdfRoot->Model();
        const sdf::ElementPtr cameraElement = sdfModel->LinkByName("link1")->SensorByIndex(0U)->Element();
        const auto& importerHook = ROS2::SDFormat::ROS2SensorHooks::ROS2CameraSensor();

        const auto& unsupportedCameraParams =
            ROS2::Utils::SDFormat::GetUnsupportedParams(cameraElement, importerHook.m_supportedSensorParams);
        EXPECT_EQ(unsupportedCameraParams.size(), 1U);
        EXPECT_EQ(unsupportedCameraParams[0U], ">pose");

        sdf::Plugin plug;
        plug.SetName("test_camera");
        plug.SetFilename("libgazebo_ros_camera.so");
        EXPECT_TRUE(ROS2::Utils::SDFormat::IsPluginSupported(plug, importerHook.m_pluginNames));
        plug.SetFilename("/usr/lib/libgazebo_ros_openni_kinect.so");
        EXPECT_TRUE(ROS2::Utils::SDFormat::IsPluginSupported(plug, importerHook.m_pluginNames));
        plug.SetFilename("~/dev/libgazebo_ros_imu.so");
        EXPECT_FALSE(ROS2::Utils::SDFormat::IsPluginSupported(plug, importerHook.m_pluginNames));
        plug.SetFilename("libgazebo_ros_camera");
        EXPECT_FALSE(ROS2::Utils::SDFormat::IsPluginSupported(plug, importerHook.m_pluginNames));

        EXPECT_TRUE(importerHook.m_sensorTypes.contains(sdf::SensorType::CAMERA));
        EXPECT_TRUE(importerHook.m_sensorTypes.contains(sdf::SensorType::DEPTH_CAMERA));
        EXPECT_FALSE(importerHook.m_sensorTypes.contains(sdf::SensorType::GPS));
    }
} // namespace UnitTest
