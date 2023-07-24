/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <AzCore/UnitTest/TestTypes.h>
#include <AzCore/std/string/string.h>
#include <AzTest/AzTest.h>
#include <RobotImporter/SDFormat/SDFormatParser.h>
#include <RobotImporter/Utils/RobotImporterUtils.h>

#include <sdf/Box.hh>
#include <sdf/sdf.hh>

namespace UnitTest
{

    class SdfParserTest : public LeakDetectionFixture
    {
    public:
        AZStd::string GetSdfWithOneLink()
        {
            return "<?xml version=\"1.0\" ?>\n"
                   "<sdf version=\"1.6\">\n"
                   "  <model name=\"test_one_link\">\n"
                   "    <pose> 0 0 0 0 0 0</pose>\n"
                   "    <link name=\"link1\">\n"
                   "      <inertial>\n"
                   "        <inertia>\n"
                   "          <ixx>1.0</ixx>\n"
                   "          <ixy>0</ixy>\n"
                   "          <ixz>0</ixz>\n"
                   "          <iyy>1.0</iyy>\n"
                   "          <iyz>0</iyz>\n"
                   "          <izz>1.0</izz>\n"
                   "        </inertia>\n"
                   "        <mass>1.0</mass>\n"
                   "      </inertial>\n"
                   "      <visual name=\"link1_viz\">\n"
                   "        <geometry>\n"
                   "          <box>\n"
                   "            <size>1.0 2.0 3.0</size>\n"
                   "          </box>\n"
                   "        </geometry>\n"
                   "        <material>\n"
                   "          <ambient>0.0 0.3 0.6 1.0</ambient>\n"
                   "        </material>\n"
                   "      </visual>\n"
                   "      <collision name=\"link1_col\">\n"
                   "        <geometry>\n"
                   "          <box>\n"
                   "            <size>1.0 2.0 3.0</size>\n"
                   "          </box>\n"
                   "        </geometry>\n"
                   "      </collision>\n"
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
                   "      </sensor>\n"
                   "    </link>\n"
                   "  </model>\n"
                   "</sdf>\n";
        }

        AZStd::string GetSdfWithTwoLinksAndJoint()
        {
            return "<?xml version=\"1.0\" ?>\n"
                   "<sdf version=\"1.6\">\n"
                   "  <model name=\"test_two_links\">\n"
                   "    <pose> 0 0 0 0 0 0</pose>\n"
                   "    <link name=\"link1\">\n"
                   "      <visual name=\"link1_viz\">\n"
                   "        <geometry>\n"
                   "          <box>\n"
                   "            <size>1.0 2.0 3.0</size>\n"
                   "          </box>\n"
                   "        </geometry>\n"
                   "      </visual>\n"
                   "      <collision name=\"link1_col\">\n"
                   "        <geometry>\n"
                   "          <box>\n"
                   "            <size>1.0 2.0 3.0</size>\n"
                   "          </box>\n"
                   "        </geometry>\n"
                   "      </collision>\n"
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
                   "      </sensor>\n"
                   "    </link>\n"
                   "    <link name=\"link2\">\n"
                   "      <visual name=\"link2_viz\">\n"
                   "        <geometry>\n"
                   "          <box>\n"
                   "            <size>1.0 2.0 3.0</size>\n"
                   "          </box>\n"
                   "        </geometry>\n"
                   "      </visual>\n"
                   "      <collision name=\"link2_col\">\n"
                   "        <geometry>\n"
                   "          <box>\n"
                   "            <size>1.0 2.0 3.0</size>\n"
                   "          </box>\n"
                   "        </geometry>\n"
                   "      </collision>\n"
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
                   "    <joint name=\"joint\" type=\"revolute\">\n"
                   "      <parent>link1</parent>\n"
                   "      <child>link2</child>\n"
                   "      <pose>0 0.5 1 0 0 0</pose>\n"
                   "      <axis>\n"
                   "        <xyz>0 0 1</xyz>\n"
                   "      </axis>\n"
                   "    </joint>\n"
                   "    <plugin name=\"joint_state\" filename=\"libgazebo_ros_joint_state_publisher.so\">\n"
                   "      <ros>\n"
                   "        <argument>~/out:=joint_states</argument>\n"
                   "      </ros>\n"
                   "      <update_rate>30</update_rate>\n"
                   "      <joint_name>joint</joint_name>\n"
                   "    </plugin>\n"
                   "  </model>\n"
                   "</sdf>\n";
        }
    };

    TEST_F(SdfParserTest, CheckModelCorrectnessOneLink)
    {
        const auto xmlStr = GetSdfWithOneLink();
        const auto sdfRoot = ROS2::SDFormatParser::Parse(xmlStr);
        const auto* sdfModel = sdfRoot->Model();

        EXPECT_EQ(sdfModel->Name(), "test_one_link");

        EXPECT_EQ(sdfModel->LinkCount(), 1U);
        const auto* link1 = sdfModel->LinkByName("link1");
        ASSERT_TRUE(link1);

        const auto& inertial = link1->Inertial();
        EXPECT_EQ(inertial.MassMatrix().Mass(), 1.0);
        EXPECT_EQ(inertial.MassMatrix().Ixx(), 1.0);
        EXPECT_EQ(inertial.MassMatrix().Ixy(), 0.0);
        EXPECT_EQ(inertial.MassMatrix().Ixz(), 0.0);
        EXPECT_EQ(inertial.MassMatrix().Iyy(), 1.0);
        EXPECT_EQ(inertial.MassMatrix().Iyz(), 0.0);
        EXPECT_EQ(inertial.MassMatrix().Izz(), 1.0);

        EXPECT_EQ(link1->VisualCount(), 1U);
        const auto visualGeometry = link1->VisualByIndex(0U)->Geom();
        ASSERT_TRUE(visualGeometry);
        EXPECT_EQ(visualGeometry->Type(), sdf::GeometryType::BOX);

        const auto material = link1->VisualByIndex(0U)->Material();
        ASSERT_TRUE(material);
        EXPECT_NEAR(material->Ambient().R(), 0.0, 1e-5);
        EXPECT_NEAR(material->Ambient().G(), 0.3, 1e-5);
        EXPECT_NEAR(material->Ambient().B(), 0.6, 1e-5);
        EXPECT_NEAR(material->Ambient().A(), 1.0, 1e-5);

        const auto* visualBox = visualGeometry->BoxShape();
        ASSERT_TRUE(visualBox);
        EXPECT_EQ(visualBox->Size().X(), 1.0);
        EXPECT_EQ(visualBox->Size().Y(), 2.0);
        EXPECT_EQ(visualBox->Size().Z(), 3.0);

        EXPECT_EQ(link1->CollisionCount(), 1U);
        const auto collisionGeometry = link1->CollisionByIndex(0U)->Geom();
        ASSERT_TRUE(collisionGeometry);
        EXPECT_EQ(collisionGeometry->Type(), sdf::GeometryType::BOX);

        const auto* collisionBox = collisionGeometry->BoxShape();
        ASSERT_TRUE(collisionBox);
        EXPECT_EQ(collisionBox->Size().X(), 1.0);
        EXPECT_EQ(collisionBox->Size().Y(), 2.0);
        EXPECT_EQ(collisionBox->Size().Z(), 3.0);

        EXPECT_EQ(link1->SensorCount(), 1U);
        const auto* sensor = link1->SensorByIndex(0U);
        ASSERT_TRUE(sensor);
        EXPECT_EQ(sensor->Type(), sdf::SensorType::CAMERA);
        EXPECT_EQ(sensor->UpdateRate(), 10);
        auto* cameraSensor = sensor->CameraSensor();
        ASSERT_TRUE(cameraSensor);
        EXPECT_EQ(cameraSensor->ImageWidth(), 640);
        EXPECT_EQ(cameraSensor->ImageHeight(), 480);
        EXPECT_NEAR(cameraSensor->HorizontalFov().Radian(), 2.0, 1e-5);
        EXPECT_NEAR(cameraSensor->NearClip(), 0.01, 1e-5);
        EXPECT_NEAR(cameraSensor->FarClip(), 1000, 1e-5);
    }

    TEST_F(SdfParserTest, CheckModelCorrectnessTwoLinks)
    {
        const auto xmlStr = GetSdfWithTwoLinksAndJoint();
        const auto sdfRoot = ROS2::SDFormatParser::Parse(xmlStr);
        const auto* sdfModel = sdfRoot->Model();

        EXPECT_EQ(sdfModel->Name(), "test_two_links");

        EXPECT_EQ(sdfModel->LinkCount(), 2U);
        const auto* link2 = sdfModel->LinkByName("link2");
        ASSERT_TRUE(link2);

        EXPECT_EQ(link2->VisualCount(), 1U);
        const auto visualGeometry = link2->VisualByIndex(0U)->Geom();
        ASSERT_TRUE(visualGeometry);
        EXPECT_EQ(visualGeometry->Type(), sdf::GeometryType::BOX);

        const auto* visualBox = visualGeometry->BoxShape();
        ASSERT_TRUE(visualBox);
        EXPECT_EQ(visualBox->Size().X(), 1.0);
        EXPECT_EQ(visualBox->Size().Y(), 2.0);
        EXPECT_EQ(visualBox->Size().Z(), 3.0);

        EXPECT_EQ(link2->CollisionCount(), 1U);
        const auto collisionGeometry = link2->CollisionByIndex(0U)->Geom();
        ASSERT_TRUE(collisionGeometry);
        EXPECT_EQ(collisionGeometry->Type(), sdf::GeometryType::BOX);

        const auto* collisionBox = collisionGeometry->BoxShape();
        ASSERT_TRUE(collisionBox);
        EXPECT_EQ(collisionBox->Size().X(), 1.0);
        EXPECT_EQ(collisionBox->Size().Y(), 2.0);
        EXPECT_EQ(collisionBox->Size().Z(), 3.0);

        EXPECT_EQ(link2->SensorCount(), 1U);
        const auto* sensor = link2->SensorByIndex(0U);
        ASSERT_TRUE(sensor);
        EXPECT_EQ(sensor->Type(), sdf::SensorType::LIDAR);
        EXPECT_EQ(sensor->UpdateRate(), 20);
        auto* lidarSensor = sensor->LidarSensor();
        ASSERT_TRUE(lidarSensor);
        EXPECT_EQ(lidarSensor->HorizontalScanSamples(), 640);
        EXPECT_NEAR(lidarSensor->HorizontalScanResolution(), 1.0, 1e-5);
        EXPECT_NEAR(lidarSensor->HorizontalScanMinAngle().Radian(), -2.0, 1e-5);
        EXPECT_NEAR(lidarSensor->HorizontalScanMaxAngle().Radian(), 2.5, 1e-5);
        EXPECT_NEAR(lidarSensor->RangeResolution(), 0.01, 1e-5);
        EXPECT_NEAR(lidarSensor->RangeMin(), 0.02, 1e-5);
        EXPECT_NEAR(lidarSensor->RangeMax(), 10.0, 1e-5);
        EXPECT_EQ(sensor->Plugins().size(), 1U);
        EXPECT_EQ(sensor->Plugins().at(0).Name(), "laser_plug");
        EXPECT_EQ(sensor->Plugins().at(0).Filename(), "librayplugin.so");

        EXPECT_EQ(sdfModel->JointCount(), 1U);
        const auto* joint = sdfModel->JointByName("joint");
        ASSERT_TRUE(joint);
        EXPECT_EQ(joint->Type(), sdf::JointType::REVOLUTE);
        EXPECT_EQ(joint->ParentName(), "link1");
        EXPECT_EQ(joint->ChildName(), "link2");
        EXPECT_NEAR(joint->RawPose().X(), 0.0, 1e-5);
        EXPECT_NEAR(joint->RawPose().Y(), 0.5, 1e-5);
        EXPECT_NEAR(joint->RawPose().Z(), 1.0, 1e-5);
        EXPECT_NEAR(joint->Axis()->Xyz().X(), 0.0, 1e-5);
        EXPECT_NEAR(joint->Axis()->Xyz().Y(), 0.0, 1e-5);
        EXPECT_NEAR(joint->Axis()->Xyz().Z(), 1.0, 1e-5);

        EXPECT_EQ(sdfModel->Plugins().size(), 1U);
        EXPECT_EQ(sdfModel->Plugins().at(0).Name(), "joint_state");
        EXPECT_EQ(sdfModel->Plugins().at(0).Filename(), "libgazebo_ros_joint_state_publisher.so");
    }

    TEST_F(SdfParserTest, CheckSensorCount)
    {
        const auto xmlStr1 = GetSdfWithOneLink();
        const auto sdfRoot1 = ROS2::SDFormatParser::Parse(xmlStr1);

        const auto& allSensors1 = ROS2::Utils::SDFormat::GetAllSensors(sdfRoot1);
        EXPECT_EQ(allSensors1.size(), 1U);
        ASSERT_TRUE(allSensors1.contains("camera"));
        EXPECT_EQ(allSensors1.at("camera")->Name(), "camera");

        const auto xmlStr2 = GetSdfWithTwoLinksAndJoint();
        const auto sdfRoot2 = ROS2::SDFormatParser::Parse(xmlStr2);

        const auto& allSensors2 = ROS2::Utils::SDFormat::GetAllSensors(sdfRoot2);
        EXPECT_EQ(allSensors2.size(), 2U);
        ASSERT_TRUE(allSensors2.contains("camera"));
        ASSERT_TRUE(allSensors2.contains("laser"));
        EXPECT_EQ(allSensors2.at("camera")->Name(), "camera");
        EXPECT_EQ(allSensors2.at("laser")->Name(), "laser");
    }

    TEST_F(SdfParserTest, CheckPluginCount)
    {
        const auto xmlStr1 = GetSdfWithOneLink();
        const auto sdfRoot1 = ROS2::SDFormatParser::Parse(xmlStr1);

        const auto& allPlugins1 = ROS2::Utils::SDFormat::GetAllPlugins(sdfRoot1);
        ASSERT_TRUE(allPlugins1.empty());

        const auto xmlStr2 = GetSdfWithTwoLinksAndJoint();
        const auto sdfRoot2 = ROS2::SDFormatParser::Parse(xmlStr2);

        const auto& allPlugins2 = ROS2::Utils::SDFormat::GetAllPlugins(sdfRoot2);
        EXPECT_EQ(allPlugins2.size(), 2U);
        ASSERT_TRUE(allPlugins2.contains("laser_plug"));
        ASSERT_TRUE(allPlugins2.contains("joint_state"));
        EXPECT_EQ(allPlugins2.at("laser_plug")->Filename(), "librayplugin.so");
        EXPECT_EQ(allPlugins2.at("joint_state")->Filename(), "libgazebo_ros_joint_state_publisher.so");
    }
} // namespace UnitTest
