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
#include <RobotImporter/URDF/UrdfParser.h>
#include <RobotImporter/Utils/RobotImporterUtils.h>
#include <RobotImporter/xacro/XacroUtils.h>

namespace UnitTest
{

    class UrdfParserTest : public LeakDetectionFixture
    {
    public:
        AZStd::string GetXacroParams()
        {
            return "<robot name=\"test\" xmlns:xacro=\"http://ros.org/wiki/xacro\">\n"
                   "    <xacro:arg name=\"laser_enabled\" default=\"false\" />\n"
                   "</robot>";
        }
        AZStd::string GetUrdfWithOneLink()
        {
            return "<robot name=\"test_one_link\">"
                   "  <material name=\"some_material\">\n"
                   "    <color rgba=\"0 0 0 1\"/>\n"
                   "  </material>"
                   "  <link name=\"link1\">"
                   "    <inertial>"
                   "      <mass value=\"1.0\"/>"
                   "      <inertia ixx=\"1.0\" iyy=\"1.0\" izz=\"1.0\" ixy=\"0\" ixz=\"0\" iyz=\"0\"/>"
                   "    </inertial>"
                   "    <visual>"
                   "      <geometry>"
                   "        <box size=\"1.0 2.0 1.0\"/>"
                   "      </geometry>"
                   "      <material name=\"some_material\"/>"
                   "    </visual>"
                   "    <collision>"
                   "      <geometry>"
                   "        <box size=\"1.0 2.0 1.0\"/>"
                   "      </geometry>"
                   "    </collision>"
                   "  </link>"
                   "</robot>";
        }

        AZStd::string GetUrdfWithTwoLinksAndJoint()
        {
            return "<robot name=\"test_two_links_one_joint\">  "
                   "  <material name=\"some_material\">\n"
                   "    <color rgba=\"0 0 0 1\"/>\n"
                   "  </material>"
                   "  <link name=\"link1\">"
                   "    <inertial>"
                   "      <mass value=\"1.0\"/>"
                   "      <inertia ixx=\"1.0\" iyy=\"1.0\" izz=\"1.0\" ixy=\"0\" ixz=\"0\" iyz=\"0\"/>"
                   "    </inertial>"
                   "    <visual>"
                   "      <geometry>"
                   "        <box size=\"1.0 2.0 1.0\"/>"
                   "      </geometry>"
                   "      <material name=\"some_material\"/>"
                   "    </visual>"
                   "  </link>"
                   "  <link name=\"link2\">"
                   "    <inertial>"
                   "      <mass value=\"1.0\"/>"
                   "      <inertia ixx=\"1.0\" iyy=\"1.0\" izz=\"1.0\" ixy=\"0\" ixz=\"0\" iyz=\"0\"/>"
                   "    </inertial>"
                   "    <visual>"
                   "      <geometry>"
                   "        <box size=\"1.0 1.0 1.0\"/>"
                   "      </geometry>"
                   "      <material name=\"some_material\"/>"
                   "    </visual>"
                   "  </link>"
                   "  <joint name=\"joint12\" type=\"fixed\">"
                   "    <parent link=\"link1\"/>"
                   "    <child link=\"link2\"/>"
                   "    <origin rpy=\"0 0 0\" xyz=\"1.0 0.5 0.0\"/>"
                   "    <dynamics damping=\"10.0\" friction=\"5.0\"/>"
                   "    <limit lower=\"10.0\" upper=\"20.0\" effort=\"90.0\" velocity=\"10.0\"/>"
                   "  </joint>"
                   "</robot>";
        }
        AZStd::string GetURDFWithTranforms()
        {
            return "<?xml version=\"1.0\"?>\n"
                   "<robot name=\"complicated\">\n"
                   "    <link name=\"base_link\">\n"
                   "    </link>\n"
                   "    <link name=\"link1\">\n"
                   "        <inertial>\n"
                   "        <origin xyz=\"0. 0. 0.\"/>\n"
                   "        <mass value=\"1.\"/>\n"
                   "        <inertia ixx=\"1.\" ixy=\"0.\" ixz=\"0.\" iyy=\"1.\" iyz=\"0.\" izz=\"1.\"/>\n"
                   "        </inertial>\n"
                   "        <visual>\n"
                   "            <origin rpy=\"0.000000 -0.000000 0\" xyz=\"-1.2 0 0\"/>\n"
                   "            <geometry>\n"
                   "                <box size=\"2.000000 0.200000 0.200000\"/>\n"
                   "            </geometry>\n"
                   "        </visual>\n"
                   "        <collision>\n"
                   "            <origin rpy=\"0.000000 -0.000000 0\" xyz=\"-1.2 0 0.000000\"/>\n"
                   "            <geometry>\n"
                   "                <box size=\"2.000000 0.200000 0.200000\"/>\n"
                   "            </geometry>            \n"
                   "        </collision>\n"
                   "    </link>\n"
                   "    <link name=\"link2\">\n"
                   "        <inertial>\n"
                   "        <origin xyz=\"0. 0. 0.\"/>\n"
                   "        <mass value=\"1.\"/>\n"
                   "        <inertia ixx=\"1.\" ixy=\"0.\" ixz=\"0.\" iyy=\"1.\" iyz=\"0.\" izz=\"1.\"/>\n"
                   "        </inertial>\n"
                   "        <visual>\n"
                   "            <origin rpy=\"0.000000 -0.000000 0\" xyz=\"-1.2 0 0\"/>\n"
                   "            <geometry>\n"
                   "                <box size=\"2.000000 0.200000 0.200000\"/>\n"
                   "            </geometry>\n"
                   "        </visual>\n"
                   "        <collision>\n"
                   "            <origin rpy=\"0.000000 -0.000000 0\" xyz=\"-1.2 0 0.000000\"/>\n"
                   "            <geometry>\n"
                   "                <box size=\"2.000000 0.200000 0.200000\"/>\n"
                   "            </geometry>            \n"
                   "        </collision>\n"
                   "    </link>\n"
                   "    <link name=\"link3\">\n"
                   "        <inertial>\n"
                   "        <origin xyz=\"0. 0. 0.\"/>\n"
                   "        <mass value=\"1.\"/>\n"
                   "        <inertia ixx=\"1.\" ixy=\"0.\" ixz=\"0.\" iyy=\"1.\" iyz=\"0.\" izz=\"1.\"/>\n"
                   "        </inertial>\n"
                   "        <visual>\n"
                   "            <origin rpy=\"0.000000 -0.000000 0\" xyz=\"-1.2 0 0\"/>\n"
                   "            <geometry>\n"
                   "                <box size=\"2.000000 0.200000 0.200000\"/>\n"
                   "            </geometry>\n"
                   "        </visual>\n"
                   "        <collision>\n"
                   "            <origin rpy=\"0.000000 -0.000000 0\" xyz=\"-1.2 0 0.000000\"/>\n"
                   "            <geometry>\n"
                   "                <box size=\"2.000000 0.200000 0.200000\"/>\n"
                   "            </geometry>            \n"
                   "        </collision>\n"
                   "    </link>\n"
                   "    <joint name=\"joint_bs\" type=\"fixed\">\n"
                   "        <parent link=\"base_link\"/>\n"
                   "        <child link=\"link1\"/>\n"
                   "        <axis xyz=\"0. 0. 1.\"/>\n"
                   "        <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>\n"
                   "    </joint> \n"
                   "    <joint name=\"joint0\" type=\"continuous\">\n"
                   "        <parent link=\"link1\"/>\n"
                   "        <child link=\"link2\"/>\n"
                   "        <axis xyz=\"0. 0. 1.\"/>\n"
                   "        <origin rpy=\"0.000000 -0.000000 2.094395\" xyz=\"-1.200000 2.078460 0.000000\"/>\n"
                   "    </joint> \n"
                   "    <joint name=\"joint1\" type=\"continuous\">\n"
                   "        <parent link=\"link2\"/>\n"
                   "        <child link=\"link3\"/>\n"
                   "        <axis xyz=\"0. 0. 1.\"/>\n"
                   "        <origin rpy=\"0.000000 0.000000 2.094395\" xyz=\"-1.200000286102295 2.078460931777954 0.\"/>\n"
                   "    </joint> \n"
                   "    <joint name=\"joint2\" type=\"continuous\">\n"
                   "        <parent link=\"link1\"/>\n"
                   "        <child link=\"link3\"/>\n"
                   "        <axis xyz=\"0. 0. 1.\"/>\n"
                   "        <origin rpy=\"0.000000 0.000000 -2.094395160675049\" xyz=\"-2.4000000953674316 0.0 0.0\"/>\n"
                   "    </joint>\n"
                   "</robot>";
        }

        AZStd::string GetURDFWithWheel(
            const AZStd::string& wheel_name,
            const AZStd::string& wheel_joint_type,
            bool wheel_has_visual = true,
            bool wheel_has_collider = true)
        {
            // clang-format off
            return "<robot name=\"wheel_test\">\n"
                   "    <link name=\"base_link\">\n"
                   "        <inertial>\n"
                   "            <origin xyz=\"0. 0. 0.\"/>\n"
                   "            <mass value=\"1.\"/>\n"
                   "            <inertia ixx=\"1.\" ixy=\"0.\" ixz=\"0.\" iyy=\"1.\" iyz=\"0.\" izz=\"1.\"/>\n"
                   "        </inertial>\n"
                   "    </link>\n"
                   "    <link name=\""+wheel_name+"\">\n"
                   "        <inertial>\n"
                   "            <origin xyz=\"0. 0. 0.\"/>\n"
                   "            <mass value=\"1.\"/>\n"
                   "            <inertia ixx=\"1.\" ixy=\"0.\" ixz=\"0.\" iyy=\"1.\" iyz=\"0.\" izz=\"1.\"/>\n"
                   "        </inertial>\n"
                   +(wheel_has_visual?"<visual>\n"
                   "            <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>\n"
                   "            <geometry>\n"
                   "                <box size=\"1 1 1\"/>\n"
                   "            </geometry>\n"
                   "        </visual>\n":"")+
                   +(wheel_has_collider?"<collision>\n"
                   "            <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>\n"
                   "            <geometry>\n"
                   "                <box size=\"1 1 1\"/>\n"
                   "            </geometry>\n"
                   "        </collision>\n":"")+
                   "    </link>\n"
                   "    <joint name=\"joint0\" type=\""+wheel_joint_type+"\">\n"
                   "        <parent link=\"base_link\"/>\n"
                   "        <child link=\""+wheel_name+"\"/>\n"
                   "        <axis xyz=\"0. 0. 1.\"/>\n"
                   "        <origin rpy=\"0. 0. 0.\" xyz=\"2. 0. 0.\"/>\n"
                   "    </joint>\n"
                   "</robot>";
            // clang-format on
        }
    };

    TEST_F(UrdfParserTest, ParseUrdfWithOneLink)
    {

        const auto xmlStr = GetUrdfWithOneLink();
        const auto urdf = ROS2::UrdfParser::Parse(xmlStr);

        EXPECT_EQ(urdf->getName(), "test_one_link");

        std::vector<urdf::LinkSharedPtr> links;
        urdf->getLinks(links);
        EXPECT_EQ(links.size(), 1U);

        const auto link1 = urdf->getLink("link1");

        ASSERT_TRUE(link1);
        EXPECT_EQ(link1->inertial->mass, 1.0);
        EXPECT_EQ(link1->inertial->ixx, 1.0);
        EXPECT_EQ(link1->inertial->ixy, 0.0);
        EXPECT_EQ(link1->inertial->ixz, 0.0);
        EXPECT_EQ(link1->inertial->iyy, 1.0);
        EXPECT_EQ(link1->inertial->iyz, 0.0);
        EXPECT_EQ(link1->inertial->izz, 1.0);

        EXPECT_EQ(link1->visual->geometry->type, 1);
        const auto visualBox = std::dynamic_pointer_cast<urdf::Box>(link1->visual->geometry);
        EXPECT_EQ(visualBox->dim.x, 1.0);
        EXPECT_EQ(visualBox->dim.y, 2.0);
        EXPECT_EQ(visualBox->dim.z, 1.0);

        EXPECT_EQ(link1->collision->geometry->type, 1);
        const auto collisionBox = std::dynamic_pointer_cast<urdf::Box>(link1->visual->geometry);
        EXPECT_EQ(collisionBox->dim.x, 1.0);
        EXPECT_EQ(collisionBox->dim.y, 2.0);
        EXPECT_EQ(collisionBox->dim.z, 1.0);
    }

    TEST_F(UrdfParserTest, ParseUrdfWithTwoLinksAndJoint)
    {

        const auto xmlStr = GetUrdfWithTwoLinksAndJoint();
        const auto urdf = ROS2::UrdfParser::Parse(xmlStr);

        EXPECT_EQ(urdf->getName(), "test_two_links_one_joint");

        std::vector<urdf::LinkSharedPtr> links;
        urdf->getLinks(links);
        EXPECT_EQ(links.size(), 2U);

        const auto link1 = urdf->getLink("link1");
        ASSERT_TRUE(link1);

        const auto link2 = urdf->getLink("link2");
        ASSERT_TRUE(link2);

        const auto joint12 = urdf->getJoint("joint12");
        ASSERT_TRUE(joint12);

        EXPECT_EQ(joint12->parent_link_name, "link1");
        EXPECT_EQ(joint12->child_link_name, "link2");

        EXPECT_EQ(joint12->parent_to_joint_origin_transform.position.x, 1.0);
        EXPECT_EQ(joint12->parent_to_joint_origin_transform.position.y, 0.5);
        EXPECT_EQ(joint12->parent_to_joint_origin_transform.position.z, 0.0);

        double roll, pitch, yaw;
        joint12->parent_to_joint_origin_transform.rotation.getRPY(roll, pitch, yaw);
        EXPECT_DOUBLE_EQ(roll, 0.0);
        EXPECT_DOUBLE_EQ(pitch, 0.0);
        EXPECT_DOUBLE_EQ(yaw, 0.0);

        EXPECT_EQ(joint12->dynamics->damping, 10.0);
        EXPECT_EQ(joint12->dynamics->friction, 5.0);

        EXPECT_EQ(joint12->limits->lower, 10.0);
        EXPECT_EQ(joint12->limits->upper, 20.0);
        EXPECT_EQ(joint12->limits->effort, 90.0);
        EXPECT_EQ(joint12->limits->velocity, 10.0);
    }

    TEST_F(UrdfParserTest, WheelHeuristicNameValid)
    {
        const AZStd::string wheel_name("wheel_left_link");
        const auto xmlStr = GetURDFWithWheel(wheel_name, "continuous");
        const auto urdf = ROS2::UrdfParser::Parse(xmlStr);
        auto wheel_candidate = urdf->getLink(wheel_name.c_str());
        ASSERT_TRUE(wheel_candidate);
        EXPECT_EQ(ROS2::Utils::IsWheelURDFHeuristics(wheel_candidate), true);
    }

    TEST_F(UrdfParserTest, WheelHeuristicNameNotValid1)
    {
        const AZStd::string wheel_name("wheel_left_joint");
        const auto xmlStr = GetURDFWithWheel(wheel_name, "continuous");
        const auto urdf = ROS2::UrdfParser::Parse(xmlStr);
        auto wheel_candidate = urdf->getLink(wheel_name.c_str());
        ASSERT_TRUE(wheel_candidate);
        EXPECT_EQ(ROS2::Utils::IsWheelURDFHeuristics(wheel_candidate), false);
    }

    TEST_F(UrdfParserTest, WheelHeuristicJointNotValid)
    {
        const AZStd::string wheel_name("wheel_left_link");
        const auto xmlStr = GetURDFWithWheel(wheel_name, "fixed");
        const auto urdf = ROS2::UrdfParser::Parse(xmlStr);
        auto wheel_candidate = urdf->getLink(wheel_name.c_str());
        ASSERT_TRUE(wheel_candidate);
        EXPECT_EQ(ROS2::Utils::IsWheelURDFHeuristics(wheel_candidate), false);
    }

    TEST_F(UrdfParserTest, WheelHeuristicJointVisualNotValid)
    {
        const AZStd::string wheel_name("wheel_left_link");
        const auto xmlStr = GetURDFWithWheel(wheel_name, "continuous", false, true);
        const auto urdf = ROS2::UrdfParser::Parse(xmlStr);
        auto wheel_candidate = urdf->getLink(wheel_name.c_str());
        ASSERT_TRUE(wheel_candidate);
        EXPECT_EQ(ROS2::Utils::IsWheelURDFHeuristics(wheel_candidate), false);
    }

    TEST_F(UrdfParserTest, WheelHeuristicJointColliderNotValid)
    {
        const AZStd::string wheel_name("wheel_left_link");
        const auto xmlStr = GetURDFWithWheel(wheel_name, "continuous", true, false);
        const auto urdf = ROS2::UrdfParser::Parse(xmlStr);
        auto wheel_candidate = urdf->getLink(wheel_name.c_str());
        ASSERT_TRUE(wheel_candidate);
        EXPECT_EQ(ROS2::Utils::IsWheelURDFHeuristics(wheel_candidate), false);
    }

    TEST_F(UrdfParserTest, TestLinkListing)
    {
        const auto xmlStr = GetURDFWithTranforms();
        const auto urdf = ROS2::UrdfParser::Parse(xmlStr);
        auto links = ROS2::Utils::GetAllLinks(urdf->getRoot()->child_links);
        EXPECT_EQ(links.size(), 3);
        ASSERT_TRUE(links.contains("link1"));
        ASSERT_TRUE(links.contains("link2"));
        ASSERT_TRUE(links.contains("link3"));
        EXPECT_EQ(links.at("link1")->name, "link1");
        EXPECT_EQ(links.at("link2")->name, "link2");
        EXPECT_EQ(links.at("link3")->name, "link3");
    }

    TEST_F(UrdfParserTest, TestJointLink)
    {
        const auto xmlStr = GetURDFWithTranforms();
        const auto urdf = ROS2::UrdfParser::Parse(xmlStr);
        auto joints = ROS2::Utils::GetAllJoints(urdf->getRoot()->child_links);
        EXPECT_EQ(joints.size(), 3);
    }

    TEST_F(UrdfParserTest, TestTransforms)
    {
        const auto xmlStr = GetURDFWithTranforms();
        const auto urdf = ROS2::UrdfParser::Parse(xmlStr);
        const auto links = ROS2::Utils::GetAllLinks(urdf->getRoot()->child_links);
        const auto link1_ptr = links.at("link1");
        const auto link2_ptr = links.at("link2");
        const auto link3_ptr = links.at("link3");

        // values exported from Blender
        const AZ::Vector3 expected_translation_link1{ 0.0, 0.0, 0.0 };
        const AZ::Vector3 expected_translation_link2{ -1.2000000476837158, 2.0784599781036377, 0.0 };
        const AZ::Vector3 expected_translation_link3{ -2.4000000953674316, 0.0, 0.0 };

        const AZ::Transform transform_from_urdf_link1 = ROS2::Utils::GetWorldTransformURDF(link1_ptr);
        EXPECT_NEAR(expected_translation_link1.GetX(), transform_from_urdf_link1.GetTranslation().GetX(), 1e-5);
        EXPECT_NEAR(expected_translation_link1.GetY(), transform_from_urdf_link1.GetTranslation().GetY(), 1e-5);
        EXPECT_NEAR(expected_translation_link1.GetZ(), transform_from_urdf_link1.GetTranslation().GetZ(), 1e-5);

        const AZ::Transform transform_from_urdf_link2 = ROS2::Utils::GetWorldTransformURDF(link2_ptr);
        EXPECT_NEAR(expected_translation_link2.GetX(), transform_from_urdf_link2.GetTranslation().GetX(), 1e-5);
        EXPECT_NEAR(expected_translation_link2.GetY(), transform_from_urdf_link2.GetTranslation().GetY(), 1e-5);
        EXPECT_NEAR(expected_translation_link2.GetZ(), transform_from_urdf_link2.GetTranslation().GetZ(), 1e-5);

        const AZ::Transform transform_from_urdf_link3 = ROS2::Utils::GetWorldTransformURDF(link3_ptr);
        EXPECT_NEAR(expected_translation_link3.GetX(), transform_from_urdf_link3.GetTranslation().GetX(), 1e-5);
        EXPECT_NEAR(expected_translation_link3.GetY(), transform_from_urdf_link3.GetTranslation().GetY(), 1e-5);
        EXPECT_NEAR(expected_translation_link3.GetZ(), transform_from_urdf_link3.GetTranslation().GetZ(), 1e-5);
    }

    TEST_F(UrdfParserTest, TestPathResolvementGlobal)
    {
        AZStd::string dae = "file:///home/foo/ros_ws/install/foo_robot/meshes/bar.dae";
        AZStd::string urdf = "/home/foo/ros_ws/install/foo_robot/foo_robot.urdf";
        auto result = ROS2::Utils::ResolveURDFPath(
            dae,
            urdf, "",
            [](const AZStd::string& p) -> bool
            {
                return false;
            });
        EXPECT_EQ(result, "/home/foo/ros_ws/install/foo_robot/meshes/bar.dae");
    }

    TEST_F(UrdfParserTest, TestPathResolvementRelative)
    {
        AZStd::string dae = "meshes/bar.dae";
        AZStd::string urdf = "/home/foo/ros_ws/install/foo_robot/foo_robot.urdf";
        auto result = ROS2::Utils::ResolveURDFPath(
            dae,
            urdf, "",
            [](const AZStd::string& p) -> bool
            {
                return false;
            });
        EXPECT_EQ(result, "/home/foo/ros_ws/install/foo_robot/meshes/bar.dae");
    }

    TEST_F(UrdfParserTest, TestPathResolvementRelativePackage)
    {
        AZStd::string dae = "package://meshes/bar.dae";
        AZStd::string urdf = "/home/foo/ros_ws/install/foo_robot/description/foo_robot.urdf";
        AZStd::string xml = "/home/foo/ros_ws/install/foo_robot/package.xml";
        AZStd::string resolvedDae = "/home/foo/ros_ws/install/foo_robot/meshes/bar.dae";
        auto mockFileSystem = [&](const AZStd::string& p) -> bool
        {
            return (p == xml || p == resolvedDae);
        };
        auto result = ROS2::Utils::ResolveURDFPath(dae, urdf, "", mockFileSystem);
        EXPECT_EQ(result, resolvedDae);
    }

    TEST_F(UrdfParserTest, TestPathResolvementExplicitPackageName)
    {
        AZStd::string dae = "package://foo_robot/meshes/bar.dae";
        AZStd::string urdf = "/home/foo/ros_ws/install/foo_robot/share/foo_robot/description/foo_robot.urdf";
        AZStd::string xml = "/home/foo/ros_ws/install/foo_robot/share/foo_robot/package.xml";
        AZStd::string resolvedDae = "/home/foo/ros_ws/install/foo_robot/share/foo_robot/meshes/bar.dae";
        auto mockFileSystem = [&](const AZStd::string& p) -> bool
        {
            return (p == xml || p == resolvedDae);
        };
        auto result = ROS2::Utils::ResolveURDFPath(dae, urdf, "/home/foo/ros_ws/install/foo_robot", mockFileSystem);
        EXPECT_EQ(result, resolvedDae);
    }

    TEST_F(UrdfParserTest, XacroParseArgs)
    {
        AZStd::string xacroParams = GetXacroParams();
        ROS2::Utils::xacro::Params params = ROS2::Utils::xacro::GetParameterFromXacroData(xacroParams);
        EXPECT_EQ(params.size(), 1);
        ASSERT_TRUE(params.contains("laser_enabled"));
        EXPECT_EQ(params["laser_enabled"], "false");
    }

} // namespace UnitTest
