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
#include <ROS2/RobotImporter/SDFormatSensorImporterHook.h>
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
            return "<?xml version=\"1.0\" ?>\n"
                   "<sdf version=\"1.6\">\n"
                   "  <model name=\"test_two_sensors\">\n"
                   "    <link name=\"link1\">\n"
                   "      <sensor name=\"camera\" type=\"camera\">\n"
                   "        <pose>0 0 0 0 0 0</pose>\n"
                   "        <camera>\n"
                   "          <horizontal_fov>2.0</horizontal_fov>\n"
                   "          <image>\n"
                   "            <width>640</width>\n"
                   "            <height>480</height>\n"
                   "          </image>\n"
                   "          <clip>\n"
                   "            <near>0.01</near>\n"
                   "            <far>1000</far>\n"
                   "          </clip>\n"
                   "        </camera>\n"
                   "        <update_rate>10</update_rate>\n"
                   "        <plugin name=\"camera_plug\" filename=\"libgazebo_ros_camera.so\">\n"
                   "          <camera_name>custom_camera</camera_name>\n"
                   "        </plugin>\n"
                   "      </sensor>\n"
                   "    </link>\n"
                   "    <link name=\"link2\">\n"
                   "      <sensor name=\"laser\" type=\"ray\">\n"
                   "        <always_on>1</always_on>\n"
                   "        <visualize>1</visualize>\n"
                   "        <update_rate>20.0</update_rate>\n"
                   "        <pose>0 0 0 0 0 0</pose>\n"
                   "        <ray>\n"
                   "          <scan>\n"
                   "            <horizontal>\n"
                   "              <samples>640</samples>\n"
                   "              <resolution>1.0</resolution>\n"
                   "              <min_angle>-2.0</min_angle>\n"
                   "              <max_angle>2.5</max_angle>\n"
                   "            </horizontal>\n"
                   "          </scan>\n"
                   "          <range>\n"
                   "            <min>0.02</min>\n"
                   "            <max>10</max>\n"
                   "            <resolution>0.01</resolution>\n"
                   "          </range>\n"
                   "        </ray>\n"
                   "        <plugin name=\"laser_plug\" filename=\"librayplugin.so\"/>\n"
                   "      </sensor>\n"
                   "    </link>\n"
                   "  </model>\n"
                   "</sdf>\n";
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

        AZStd::unordered_set<AZStd::string> cameraSupportedOptions{
            ">update_rate", ">camera>horizontal_fov", ">camera>image>width", ">camera>image>height"
        };

        const auto xmlStr = GetSdfWithTwoSensors();
        const auto sdfRoot = Parse(xmlStr);
        const auto* sdfModel = sdfRoot->Model();
        const sdf::ElementPtr cameraElement = sdfModel->LinkByName("link1")->SensorByIndex(0U)->Element();
        const sdf::ElementPtr laserElement = sdfModel->LinkByName("link2")->SensorByIndex(0U)->Element();

        {
            const auto& unsupportedCameraOptions = ROS2::Utils::SDFormat::GetUnsupportedOptions(cameraElement, cameraSupportedOptions);
            EXPECT_EQ(unsupportedCameraOptions.size(), 3U);
            EXPECT_EQ(unsupportedCameraOptions[0U], ">pose");
            EXPECT_EQ(unsupportedCameraOptions[1U], ">camera>clip>near");
            EXPECT_EQ(unsupportedCameraOptions[2U], ">camera>clip>far");
        }

        cameraSupportedOptions.emplace(">pose");
        {
            const auto& unsupportedCameraOptions = ROS2::Utils::SDFormat::GetUnsupportedOptions(cameraElement, cameraSupportedOptions);
            EXPECT_EQ(unsupportedCameraOptions.size(), 2U);
            EXPECT_EQ(unsupportedCameraOptions[0U], ">camera>clip>near");
            EXPECT_EQ(unsupportedCameraOptions[1U], ">camera>clip>far");
        }

        cameraSupportedOptions.emplace(">camera>clip>near");
        cameraSupportedOptions.emplace(">camera>clip>far");
        {
            const auto& unsupportedCameraOptions = ROS2::Utils::SDFormat::GetUnsupportedOptions(cameraElement, cameraSupportedOptions);
            EXPECT_EQ(unsupportedCameraOptions.size(), 0U);
        }

        const AZStd::unordered_set<AZStd::string> laserSupportedOptions{ ">pose",
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
        const auto& unsupportedLaserOptions = ROS2::Utils::SDFormat::GetUnsupportedOptions(laserElement, laserSupportedOptions);
        EXPECT_EQ(unsupportedLaserOptions.size(), 0U);
    }

    TEST_F(SdfParserTest, SensorPluginImporterHookCheck)
    {
        const auto xmlStr = GetSdfWithTwoSensors();
        const auto sdfRoot = Parse(xmlStr);
        const auto* sdfModel = sdfRoot->Model();
        const sdf::ElementPtr cameraElement = sdfModel->LinkByName("link1")->SensorByIndex(0U)->Element();
        const auto& importerHook = ROS2::SDFormat::ROS2SensorHooks::ROS2CameraSensor();

        const auto& unsupportedCameraOptions = ROS2::Utils::SDFormat::GetUnsupportedOptions(cameraElement, importerHook.m_sensorOptions);
        EXPECT_EQ(unsupportedCameraOptions.size(), 1U);
        EXPECT_EQ(unsupportedCameraOptions[0U], ">pose");

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
