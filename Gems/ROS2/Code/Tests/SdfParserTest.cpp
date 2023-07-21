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
        AZStd::string GetSdfWithOneLlinkOneSensor()
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
    };

    TEST_F(SdfParserTest, CheckModelCorrectness)
    {
        const auto xmlStr = GetSdfWithOneLlinkOneSensor();
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

    TEST_F(SdfParserTest, CheckSensorCount)
    {
        const auto xmlStr = GetSdfWithOneLlinkOneSensor();
        const auto sdfRoot = ROS2::SDFormatParser::Parse(xmlStr);

        const auto& allSensors = ROS2::Utils::SDFormat::GetAllSensors(sdfRoot);
        EXPECT_EQ(allSensors.size(), 1U);
        ASSERT_TRUE(allSensors.contains("camera"));
        EXPECT_EQ(allSensors.at("camera")->Name(), "camera");
    }
} // namespace UnitTest
