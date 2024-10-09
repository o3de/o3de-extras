/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <AzCore/UnitTest/TestTypes.h>
#include <AzCore/std/ranges/ranges_algorithm.h>
#include <AzCore/std/string/string.h>
#include <AzTest/AzTest.h>
#include <RobotImporter/URDF/UrdfParser.h>
#include <RobotImporter/Utils/ErrorUtils.h>
#include <RobotImporter/Utils/RobotImporterUtils.h>
#include <RobotImporter/xacro/XacroUtils.h>
#include <SdfAssetBuilder/SdfAssetBuilderSettings.h>

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

        AZStd::string GetUrdfWithTwoLinksAndJoint(AZStd::string_view jointType = "fixed")
        {
            return AZStd::string::format(
                "<robot name=\"test_two_links_one_joint\">  "
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
                R"(  <joint name="joint12" type="%.*s">)"
                "    <parent link=\"link1\"/>"
                "    <child link=\"link2\"/>"
                "    <origin rpy=\"0 0 0\" xyz=\"1.0 0.5 0.0\"/>"
                "    <dynamics damping=\"10.0\" friction=\"5.0\"/>"
                "    <limit lower=\"10.0\" upper=\"20.0\" effort=\"90.0\" velocity=\"10.0\"/>"
                "  </joint>"
                "</robot>",
                AZ_STRING_ARG(jointType));
        }

        AZStd::string GetURDFWithTwoLinksAndBaseLinkNoInertia()
        {
            return R"(<?xml version="1.0" ?>
                <robot name="always_ignored">
                  <link name="base_link">
                    <visual>
                      <geometry>
                        <box size="1.0 1.0 1.0"/>
                      </geometry>
                    </visual>
                  </link>
                  <link name="child_link">
                    <inertial>
                      <mass value="1.0"/>
                      <inertia ixx="1.0" iyy="1.0" izz="1.0" ixy="0" ixz="0" iyz="0"/>
                    </inertial>
                    <visual>
                      <geometry>
                        <sphere radius="1.0"/>
                      </geometry>
                    </visual>
                  </link>
                  <joint name="joint12" type="fixed">
                    <parent link="base_link"/>
                    <child link="child_link"/>
                  </joint>
                </robot>)";
        }

        AZStd::string GetURDFWithFourLinksAndRootLinkNoInertia(AZStd::string_view rootLinkName)
        {
            return AZStd::string::format(
                R"(<?xml version="1.0" ?>
                <robot name="FooRobot">
                  <link name="%.*s"/>
                  <link name="base_link"/>
                  <link name="base_link_inertia">
                    <inertial>
                      <mass value="1.0"/>
                      <inertia ixx="1.0" iyy="1.0" izz="1.0" ixy="0" ixz="0" iyz="0"/>
                    </inertial>
                    <visual>
                      <geometry>
                        <box size="1.0 1.0 1.0"/>
                      </geometry>
                    </visual>
                  </link>
                  <link name="child_link">
                    <inertial>
                      <mass value="2.0"/>
                      <inertia ixx="1.0" iyy="1.0" izz="1.0" ixy="0" ixz="0" iyz="0"/>
                    </inertial>
                    <visual>
                      <geometry>
                        <sphere radius="1.0"/>
                      </geometry>
                    </visual>
                  </link>
                  <joint name="world_base_joint" type="fixed">
                    <parent link="%.*s"/>
                    <child link="base_link"/>
                  </joint>
                  <joint name="base_inertia_joint" type="fixed">
                    <parent link="base_link"/>
                    <child link="base_link_inertia"/>
                  </joint>
                  <joint name="base_inertia_child_joint" type="revolute">
                    <parent link="base_link_inertia"/>
                    <child link="child_link"/>
                    <axis xyz="0 0 1" />
                    <origin rpy="0 0 0" xyz="1.0 0.5 0.0"/>
                    <dynamics damping="10.0" friction="5.0"/>
                    <limit lower="10.0" upper="20.0" effort="90.0" velocity="10.0"/>
                  </joint>
                </robot>)",
                AZ_STRING_ARG(rootLinkName),
                AZ_STRING_ARG(rootLinkName));
        }

        // A URDF <model> can only represent structure which is configurable though the <joint> tags
        // Therefore link can only appear as a child of a single joint.
        // It cannot be a child of multiple joints
        // https://wiki.ros.org/urdf/XML/model
        AZStd::string GetURDFWithTransforms()
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

        ROS2::SdfAssetBuilderSettings GetTestSettings()
        {
            ROS2::SdfAssetBuilderSettings settings;
            settings.m_resolverSettings.m_useAmentPrefixPath = true;
            settings.m_resolverSettings.m_useAncestorPaths = true;
            settings.m_resolverSettings.m_uriPrefixMap.emplace("model://", AZStd::vector<AZStd::string>({"."}));
            settings.m_resolverSettings.m_uriPrefixMap.emplace("package://", AZStd::vector<AZStd::string>({"."}));
            settings.m_resolverSettings.m_uriPrefixMap.emplace("file://", AZStd::vector<AZStd::string>({"."}));

            return settings;
        }
    };

    TEST_F(UrdfParserTest, ParseUrdfWithOneLink)
    {
        const auto xmlStr = GetUrdfWithOneLink();
        sdf::ParserConfig parserConfig;
        const auto sdfRootOutcome = ROS2::UrdfParser::Parse(xmlStr, parserConfig);
        ASSERT_TRUE(sdfRootOutcome);
        const sdf::Root& sdfRoot = sdfRootOutcome.GetRoot();

        const sdf::Model* model = sdfRoot.Model();
        EXPECT_EQ("test_one_link", model->Name());
        ASSERT_NE(nullptr, model);

        uint64_t linkCount = model->LinkCount();
        EXPECT_EQ(1U, linkCount);

        const sdf::Link* link1 = model->LinkByName("link1");

        ASSERT_NE(nullptr, link1);
        EXPECT_EQ(1.0, link1->Inertial().MassMatrix().Mass());
        EXPECT_EQ(1.0, link1->Inertial().MassMatrix().Ixx());
        EXPECT_EQ(0.0, link1->Inertial().MassMatrix().Ixy());
        EXPECT_EQ(0.0, link1->Inertial().MassMatrix().Ixz());
        EXPECT_EQ(1.0, link1->Inertial().MassMatrix().Iyy());
        EXPECT_EQ(0.0, link1->Inertial().MassMatrix().Iyz());
        EXPECT_EQ(1.0, link1->Inertial().MassMatrix().Izz());

        ASSERT_EQ(1, link1->VisualCount());
        const sdf::Visual* visual = link1->VisualByIndex(0);
        ASSERT_NE(nullptr, visual);
        const sdf::Geometry* geometryVis = visual->Geom();
        ASSERT_NE(nullptr, geometryVis);
        EXPECT_EQ(sdf::GeometryType::BOX, geometryVis->Type());
        const sdf::Box* visualBox = geometryVis->BoxShape();
        ASSERT_NE(nullptr, visualBox);
        EXPECT_EQ(1.0, visualBox->Size().X());
        EXPECT_EQ(2.0, visualBox->Size().Y());
        EXPECT_EQ(1.0, visualBox->Size().Z());

        ASSERT_EQ(1, link1->CollisionCount());
        const sdf::Collision* collision = link1->CollisionByIndex(0);
        ASSERT_NE(nullptr, collision);
        const sdf::Geometry* geometryCol = visual->Geom();
        ASSERT_NE(nullptr, geometryCol);
        EXPECT_EQ(sdf::GeometryType::BOX, geometryCol->Type());
        const sdf::Box* collisionBox = geometryCol->BoxShape();
        EXPECT_EQ(1.0, collisionBox->Size().X());
        EXPECT_EQ(2.0, collisionBox->Size().Y());
        EXPECT_EQ(1.0, collisionBox->Size().Z());
    }

    TEST_F(UrdfParserTest, ParseUrdfWithTwoLinksAndFixedJoint_WithPreserveFixedJoint_False)
    {
        const auto xmlStr = GetUrdfWithTwoLinksAndJoint();
        sdf::ParserConfig parserConfig;
        const auto sdfRootOutcome = ROS2::UrdfParser::Parse(xmlStr, parserConfig);
        ASSERT_TRUE(sdfRootOutcome);
        const sdf::Root& sdfRoot = sdfRootOutcome.GetRoot();

        const sdf::Model* model = sdfRoot.Model();
        EXPECT_EQ("test_two_links_one_joint", model->Name());
        ASSERT_NE(nullptr, model);

        // The SDFormat URDF parser combines links in joints that are fixed
        // together
        // https://github.com/gazebosim/sdformat/pull/1149
        // So for a URDF with 2 links that are combined with a single fixed joint
        // The resulted SDF has one 1 link and no joints
        //
        // The SDFormat <gazebo> extension tag can be used to preserve fixed joint by adding
        // a <gazebo><preserveFixedJoint>true</preserveFixedJoint></gazebo> XML element
        // http://sdformat.org/tutorials?tut=sdformat_urdf_extensions&cat=specification&#gazebo-elements-for-joints
        ASSERT_EQ(1, model->LinkCount());

        EXPECT_TRUE(model->FrameNameExists("link2"));
        EXPECT_TRUE(model->FrameNameExists("joint12"));

        const sdf::Link* link1 = model->LinkByName("link1");
        ASSERT_NE(nullptr, link1);
    }

    TEST_F(UrdfParserTest, ParseUrdfWithTwoLinksAndFixedJoint_WithPreserveFixedJoint_True)
    {
        const auto xmlStr = GetUrdfWithTwoLinksAndJoint();
        sdf::ParserConfig parserConfig;
        parserConfig.URDFSetPreserveFixedJoint(true);
        const auto sdfRootOutcome = ROS2::UrdfParser::Parse(xmlStr, parserConfig);
        ASSERT_TRUE(sdfRootOutcome);
        const sdf::Root& sdfRoot = sdfRootOutcome.GetRoot();

        const sdf::Model* model = sdfRoot.Model();
        EXPECT_EQ("test_two_links_one_joint", model->Name());
        ASSERT_NE(nullptr, model);

        // As the <preserveFixedJoint> option has been set
        // in this case the child link and the joint in the SDF should be remain
        ASSERT_EQ(2, model->LinkCount());
        ASSERT_EQ(1, model->JointCount());

        // No Frames are made for perserved joints
        EXPECT_FALSE(model->FrameNameExists("link2"));
        EXPECT_FALSE(model->FrameNameExists("joint12"));

        const sdf::Link* link1 = model->LinkByName("link1");
        ASSERT_NE(nullptr, link1);

        const sdf::Link* link2 = model->LinkByName("link2");
        ASSERT_NE(nullptr, link2);

        const sdf::Joint* joint12 = model->JointByName("joint12");
        ASSERT_NE(nullptr, joint12);

        EXPECT_EQ("link1", joint12->ParentName());
        EXPECT_EQ("link2", joint12->ChildName());

        gz::math::Pose3d jointPose = joint12->RawPose();
        EXPECT_DOUBLE_EQ(1.0, jointPose.X());
        EXPECT_DOUBLE_EQ(0.5, jointPose.Y());
        EXPECT_DOUBLE_EQ(0.0, jointPose.Z());

        double roll, pitch, yaw;
        const gz::math::Quaternion rot = jointPose.Rot();
        roll = rot.Roll();
        pitch = rot.Pitch();
        yaw = rot.Yaw();
        EXPECT_DOUBLE_EQ(roll, 0.0);
        EXPECT_DOUBLE_EQ(pitch, 0.0);
        EXPECT_DOUBLE_EQ(yaw, 0.0);

        EXPECT_EQ(sdf::JointType::FIXED, joint12->Type());
    }

    TEST_F(UrdfParserTest, ParseUrdfWithTwoLinksAndNonFixedJoint)
    {
        const auto xmlStr = GetUrdfWithTwoLinksAndJoint("continuous");
        sdf::ParserConfig parserConfig;
        const auto sdfRootOutcome = ROS2::UrdfParser::Parse(xmlStr, parserConfig);
        ASSERT_TRUE(sdfRootOutcome);
        const sdf::Root& sdfRoot = sdfRootOutcome.GetRoot();

        const sdf::Model* model = sdfRoot.Model();
        EXPECT_EQ("test_two_links_one_joint", model->Name());
        ASSERT_NE(nullptr, model);

        ASSERT_EQ(2, model->LinkCount());

        const sdf::Link* link1 = model->LinkByName("link1");
        ASSERT_NE(nullptr, link1);

        const sdf::Link* link2 = model->LinkByName("link2");
        ASSERT_NE(nullptr, link2);

        const sdf::Joint* joint12 = model->JointByName("joint12");
        ASSERT_NE(nullptr, joint12);

        EXPECT_EQ("link1", joint12->ParentName());
        EXPECT_EQ("link2", joint12->ChildName());

        gz::math::Pose3d jointPose = joint12->RawPose();
        EXPECT_DOUBLE_EQ(1.0, jointPose.X());
        EXPECT_DOUBLE_EQ(0.5, jointPose.Y());
        EXPECT_DOUBLE_EQ(0.0, jointPose.Z());

        double roll, pitch, yaw;
        const gz::math::Quaternion rot = jointPose.Rot();
        roll = rot.Roll();
        pitch = rot.Pitch();
        yaw = rot.Yaw();
        EXPECT_DOUBLE_EQ(roll, 0.0);
        EXPECT_DOUBLE_EQ(pitch, 0.0);
        EXPECT_DOUBLE_EQ(yaw, 0.0);

        const sdf::JointAxis* joint12Axis = joint12->Axis();
        ASSERT_NE(nullptr, joint12Axis);

        EXPECT_DOUBLE_EQ(10.0, joint12Axis->Damping());
        EXPECT_DOUBLE_EQ(5.0, joint12Axis->Friction());

        EXPECT_DOUBLE_EQ(-AZStd::numeric_limits<double>::infinity(), joint12Axis->Lower());
        EXPECT_DOUBLE_EQ(AZStd::numeric_limits<double>::infinity(), joint12Axis->Upper());
        EXPECT_DOUBLE_EQ(90.0, joint12Axis->Effort());
        EXPECT_DOUBLE_EQ(10.0, joint12Axis->MaxVelocity());
    }

    TEST_F(UrdfParserTest, ParseURDF_WithTwoLinks_AndBaseLinkWithNoInertia_WithUrdfFixedJointPreservationOn_Fails)
    {
        const auto xmlStr = GetURDFWithTwoLinksAndBaseLinkNoInertia();
        sdf::ParserConfig parserConfig;
        parserConfig.URDFSetPreserveFixedJoint(true);
        const auto sdfRootOutcome = ROS2::UrdfParser::Parse(xmlStr, parserConfig);
        ASSERT_FALSE(sdfRootOutcome);
        AZStd::string errorString = ROS2::Utils::JoinSdfErrorsToString(sdfRootOutcome.GetSdfErrors());
        printf("URDF with single link and no inertia: %s\n", errorString.c_str());
    }

    TEST_F(UrdfParserTest, ParseURDF_WithTwoLinks_AndBaseLinkWithNoInertia_WithUrdfFixedJointPreservationOff_Succeeds)
    {
        const auto xmlStr = GetURDFWithTwoLinksAndBaseLinkNoInertia();
        sdf::ParserConfig parserConfig;
        parserConfig.URDFSetPreserveFixedJoint(false);
        const auto sdfRootOutcome = ROS2::UrdfParser::Parse(xmlStr, parserConfig);
        ASSERT_TRUE(sdfRootOutcome);
        const sdf::Root& sdfRoot = sdfRootOutcome.GetRoot();

        const sdf::Model* model = sdfRoot.Model();
        EXPECT_EQ("always_ignored", model->Name());
        ASSERT_NE(nullptr, model);

        ASSERT_EQ(1, model->LinkCount());

        EXPECT_TRUE(model->FrameNameExists("child_link"));
        EXPECT_TRUE(model->FrameNameExists("joint12"));

        const sdf::Link* link1 = model->LinkByName("base_link");
        ASSERT_NE(nullptr, link1);
    }

    MATCHER(UnorderedMapKeyMatcher, "") {
        *result_listener << "Expected Key" << AZStd::get<1>(arg)
            << "Actual Key" << AZStd::get<0>(arg).first.c_str();
        return AZStd::get<0>(arg).first == AZStd::get<1>(arg);
    }

    TEST_F(UrdfParserTest, ParseUrdf_WithRootLink_WithName_world_DoesNotContain_world_Link)
    {
        // The libsdformat URDF parser skips converting the root link if its name is "world"
        // https://github.com/gazebosim/sdformat/blob/a1027c3ed96f2f663760df10f13b06f47f922c55/src/parser_urdf.cc#L3385-L3399
        // Therefore it will not be part of joint reduction
        constexpr const char* RootLinkName = "world";
        const auto xmlStr = GetURDFWithFourLinksAndRootLinkNoInertia(RootLinkName);
        sdf::ParserConfig parserConfig;
        // Make sure joint reduction occurs
        parserConfig.URDFSetPreserveFixedJoint(false);
        const auto sdfRootOutcome = ROS2::UrdfParser::Parse(xmlStr, parserConfig);
        ASSERT_TRUE(sdfRootOutcome);
        const sdf::Root& sdfRoot = sdfRootOutcome.GetRoot();

        const sdf::Model* model = sdfRoot.Model();
        EXPECT_EQ("FooRobot", model->Name());
        ASSERT_NE(nullptr, model);

        // Due to joint reduction there should be 2 links and 2 frames
        ASSERT_EQ(2, model->LinkCount());
        ASSERT_EQ(2, model->FrameCount());

        // The base_inertia_joint was elimimated when the base_link parent
        // merged with the base_link_inertia child
        // NOTE: The "world_base_joint" is not merged because the root link
        // is has a name of "world"
        EXPECT_TRUE(model->FrameNameExists("base_inertia_joint"));
        EXPECT_TRUE(model->FrameNameExists("base_link_inertia"));

        const sdf::Link* link1 = model->LinkByName("base_link");
        ASSERT_NE(nullptr, link1);

        const sdf::Link* link2 = model->LinkByName("child_link");
        ASSERT_NE(nullptr, link2);

        // Check the ROS2 visitor logic to make sure the joint with "world" parent link isn't visited
        auto joints = ROS2::Utils::GetAllJoints(*model);
        EXPECT_EQ(1, joints.size());
        EXPECT_THAT(joints, ::testing::UnorderedPointwise(UnorderedMapKeyMatcher(), { "FooRobot::base_inertia_child_joint" }));

        // The libsdformat sdf::Model::JointCount function however returns 2
        // as the "world_base_joint" is skipped over by joint reduction
        // because the it's parent link is "world"
        EXPECT_EQ(2, model->JointCount());
    }

    TEST_F(UrdfParserTest, ParseUrdf_WithRootLink_WithoutName_world_DoesContainThatLink)
    {
        // Because the name of the root link in the URDF is not "world"
        // It should get reduced as part of joint reduction
        constexpr const char* RootLinkName = "not_world";
        const auto xmlStr = GetURDFWithFourLinksAndRootLinkNoInertia(RootLinkName);
        sdf::ParserConfig parserConfig;
        // Make sure joint reduction occurs
        parserConfig.URDFSetPreserveFixedJoint(false);
        const auto sdfRootOutcome = ROS2::UrdfParser::Parse(xmlStr, parserConfig);
        ASSERT_TRUE(sdfRootOutcome);
        const sdf::Root& sdfRoot = sdfRootOutcome.GetRoot();

        const sdf::Model* model = sdfRoot.Model();
        EXPECT_EQ("FooRobot", model->Name());
        ASSERT_NE(nullptr, model);

        // Due to joint reduction there should be 2 links and 4 frames
        // This is different from the previous test, as the root link
        // can now participate in joint reduction because its name isn't "world"
        ASSERT_EQ(2, model->LinkCount());
        ASSERT_EQ(4, model->FrameCount());

        // The base_inertia_joint and world_base_joint should be merged
        // together into the top level link of "not_world"
        EXPECT_TRUE(model->FrameNameExists("base_inertia_joint"));
        EXPECT_TRUE(model->FrameNameExists("base_link_inertia"));
        EXPECT_TRUE(model->FrameNameExists("world_base_joint"));
        EXPECT_TRUE(model->FrameNameExists("base_link"));

        const sdf::Link* link1 = model->LinkByName(RootLinkName);
        ASSERT_NE(nullptr, link1);

        const sdf::Link* link2 = model->LinkByName("child_link");
        ASSERT_NE(nullptr, link2);

        // The ROS2 Visitor logic should visit all reduced joints which
        // there should only be a single one of the revolute joint
        auto joints = ROS2::Utils::GetAllJoints(*model);
        EXPECT_EQ(1, joints.size());
        EXPECT_THAT(joints, ::testing::UnorderedPointwise(UnorderedMapKeyMatcher(), { "FooRobot::base_inertia_child_joint" }));

        // The libsdformat sdf::Model::JointCount function should match
        // the ROS2 Visitor logic as the root link of "not_world" is part of joint reduction
        EXPECT_EQ(1, model->JointCount());
    }

    TEST_F(UrdfParserTest, WheelHeuristicNameValid)
    {
        sdf::ParserConfig parserConfig;
        const AZStd::string_view wheelName("wheel_left_link");
        const auto xmlStr = GetURDFWithWheel(wheelName, "continuous");
        const auto sdfRootOutcome = ROS2::UrdfParser::Parse(xmlStr, parserConfig);
        ASSERT_TRUE(sdfRootOutcome);
        const sdf::Root& sdfRoot = sdfRootOutcome.GetRoot();
        const sdf::Model* model = sdfRoot.Model();
        ASSERT_NE(nullptr, model);
        auto wheelCandidate = model->LinkByName(std::string(wheelName.data(), wheelName.size()));
        ASSERT_NE(nullptr, wheelCandidate);
        EXPECT_TRUE(ROS2::Utils::IsWheelURDFHeuristics(*model, wheelCandidate));

        const AZStd::string_view wheelNameSuffix("left_link_wheel");
        const auto xmlStr2 = GetURDFWithWheel(wheelNameSuffix, "continuous");
        const auto sdfRootOutcome2 = ROS2::UrdfParser::Parse(xmlStr2, parserConfig);
        ASSERT_TRUE(sdfRootOutcome2);
        const sdf::Root& sdfRoot2 = sdfRootOutcome2.GetRoot();
        const sdf::Model* model2 = sdfRoot2.Model();
        ASSERT_NE(nullptr, model2);
        auto wheelCandidate2 = model2->LinkByName(std::string(wheelNameSuffix.data(), wheelNameSuffix.size()));
        ASSERT_NE(nullptr, wheelCandidate2);
        EXPECT_TRUE(ROS2::Utils::IsWheelURDFHeuristics(*model2, wheelCandidate2));
    }

    TEST_F(UrdfParserTest, WheelHeuristicNameNotValid1)
    {
        const AZStd::string wheelName("whe3l_left_link");
        const auto xmlStr = GetURDFWithWheel(wheelName, "continuous");
        sdf::ParserConfig parserConfig;
        const auto sdfRootOutcome = ROS2::UrdfParser::Parse(xmlStr, parserConfig);
        ASSERT_TRUE(sdfRootOutcome);
        const sdf::Root& sdfRoot = sdfRootOutcome.GetRoot();
        const sdf::Model* model = sdfRoot.Model();
        ASSERT_NE(nullptr, model);
        auto wheelCandidate = model->LinkByName(std::string(wheelName.data(), wheelName.size()));
        ASSERT_NE(nullptr, wheelCandidate);
        EXPECT_FALSE(ROS2::Utils::IsWheelURDFHeuristics(*model, wheelCandidate));
    }

    TEST_F(UrdfParserTest, WheelHeuristicJointNotValid)
    {
        const AZStd::string wheelName("wheel_left_link");
        const auto xmlStr = GetURDFWithWheel(wheelName, "fixed");
        sdf::ParserConfig parserConfig;
        const auto sdfRootOutcome = ROS2::UrdfParser::Parse(xmlStr, parserConfig);
        ASSERT_TRUE(sdfRootOutcome);
        const sdf::Root& sdfRoot = sdfRootOutcome.GetRoot();
        const sdf::Model* model = sdfRoot.Model();
        ASSERT_NE(nullptr, model);
        // SDFormat converts combines the links of a joint with a fixed type
        // into a single link
        // It does however create a Frame with the name of the child link and joint that was combined
        EXPECT_EQ(1, model->LinkCount());

        auto wheelCandidate = model->LinkByName("base_link");
        ASSERT_NE(nullptr, wheelCandidate);

        EXPECT_TRUE(model->FrameNameExists(std::string{ wheelName.c_str(), wheelName.size() }));
        EXPECT_TRUE(model->FrameNameExists("joint0"));
        EXPECT_FALSE(ROS2::Utils::IsWheelURDFHeuristics(*model, wheelCandidate));
    }

    TEST_F(UrdfParserTest, WheelHeuristicJointVisualNotValid)
    {
        const AZStd::string wheelName("wheel_left_link");
        const auto xmlStr = GetURDFWithWheel(wheelName, "continuous", false, true);
        sdf::ParserConfig parserConfig;
        const auto sdfRootOutcome = ROS2::UrdfParser::Parse(xmlStr, parserConfig);
        ASSERT_TRUE(sdfRootOutcome);
        const sdf::Root& sdfRoot = sdfRootOutcome.GetRoot();
        const sdf::Model* model = sdfRoot.Model();
        ASSERT_NE(nullptr, model);
        auto wheelCandidate = model->LinkByName(std::string(wheelName.c_str(), wheelName.size()));
        ASSERT_NE(nullptr, wheelCandidate);
        EXPECT_FALSE(ROS2::Utils::IsWheelURDFHeuristics(*model, wheelCandidate));
    }

    TEST_F(UrdfParserTest, WheelHeuristicJointColliderNotValid)
    {
        const AZStd::string wheelName("wheel_left_link");
        const auto xmlStr = GetURDFWithWheel(wheelName, "continuous", true, false);
        sdf::ParserConfig parserConfig;
        const auto sdfRootOutcome = ROS2::UrdfParser::Parse(xmlStr, parserConfig);
        ASSERT_TRUE(sdfRootOutcome);
        const sdf::Root& sdfRoot = sdfRootOutcome.GetRoot();
        const sdf::Model* model = sdfRoot.Model();
        ASSERT_NE(nullptr, model);
        auto wheelCandidate = model->LinkByName(std::string(wheelName.c_str(), wheelName.size()));
        ASSERT_NE(nullptr, wheelCandidate);
        EXPECT_FALSE(ROS2::Utils::IsWheelURDFHeuristics(*model, wheelCandidate));
    }

    TEST_F(UrdfParserTest, TestLinkListing)
    {
        const auto xmlStr = GetURDFWithTransforms();
        sdf::ParserConfig parserConfig;
        const auto sdfRootOutcome = ROS2::UrdfParser::Parse(xmlStr, parserConfig);
        ASSERT_TRUE(sdfRootOutcome);
        const sdf::Root& sdfRoot = sdfRootOutcome.GetRoot();
        const sdf::Model* model = sdfRoot.Model();
        ASSERT_NE(nullptr, model);
        auto links = ROS2::Utils::GetAllLinks(*model);
        // As the "joint_bs" is a fixed joint, it and it's child link are combined
        // Therefore the "link1" child link and "joint_bs" fixed joint are combined
        // into the base_link of the SDF
        // However there are Frames for the combined links and joints
        EXPECT_EQ(links.size(), 3);
        ASSERT_TRUE(links.contains("complicated::base_link"));
        ASSERT_TRUE(links.contains("complicated::link2"));
        ASSERT_TRUE(links.contains("complicated::link3"));
        EXPECT_EQ("base_link", links.at("complicated::base_link")->Name());
        EXPECT_EQ("link2", links.at("complicated::link2")->Name());
        EXPECT_EQ("link3", links.at("complicated::link3")->Name());

        // Check that the frame names exist on the model
        EXPECT_TRUE(model->FrameNameExists("joint_bs"));
        EXPECT_TRUE(model->FrameNameExists("link1"));
    }

    TEST_F(UrdfParserTest, TestJointLink)
    {
        const auto xmlStr = GetURDFWithTransforms();
        sdf::ParserConfig parserConfig;
        const auto sdfRootOutcome = ROS2::UrdfParser::Parse(xmlStr, parserConfig);
        ASSERT_TRUE(sdfRootOutcome);
        const sdf::Root& sdfRoot = sdfRootOutcome.GetRoot();
        const sdf::Model* model = sdfRoot.Model();
        ASSERT_NE(nullptr, model);
        auto joints = ROS2::Utils::GetAllJoints(*model);
        EXPECT_EQ(2, joints.size());
        ASSERT_TRUE(joints.contains("complicated::joint0"));
        ASSERT_TRUE(joints.contains("complicated::joint1"));
    }

    TEST_F(UrdfParserTest, TestTransforms)
    {
        const auto xmlStr = GetURDFWithTransforms();
        sdf::ParserConfig parserConfig;
        const auto sdfRootOutcome = ROS2::UrdfParser::Parse(xmlStr, parserConfig);
        ASSERT_TRUE(sdfRootOutcome);
        const sdf::Root& sdfRoot = sdfRootOutcome.GetRoot();
        const sdf::Model* model = sdfRoot.Model();
        ASSERT_NE(nullptr, model);
        const auto links = ROS2::Utils::GetAllLinks(*model);
        // The "link1" is combined with the base_link through
        // joint reduction in the URDF->SDF parser logic
        // https://github.com/gazebosim/sdformat/issues/1110
        ASSERT_TRUE(links.contains("complicated::base_link"));
        ASSERT_TRUE(links.contains("complicated::link2"));
        ASSERT_TRUE(links.contains("complicated::link3"));
        const auto base_link_ptr = links.at("complicated::base_link");
        const auto link2_ptr = links.at("complicated::link2");
        const auto link3_ptr = links.at("complicated::link3");

        // values exported from Blender
        const AZ::Vector3 expected_translation_link1{ 0.0, 0.0, 0.0 };
        const AZ::Vector3 expected_translation_link2{ -1.2000000476837158, 2.0784599781036377, 0.0 };
        const AZ::Vector3 expected_translation_link3{ -2.4000000953674316, 0.0, 0.0 };

        const auto base_link_pose = base_link_ptr->SemanticPose();
        const AZ::Transform transform_from_urdf_link1 = ROS2::Utils::GetLocalTransformURDF(base_link_pose);
        EXPECT_NEAR(expected_translation_link1.GetX(), transform_from_urdf_link1.GetTranslation().GetX(), 1e-5);
        EXPECT_NEAR(expected_translation_link1.GetY(), transform_from_urdf_link1.GetTranslation().GetY(), 1e-5);
        EXPECT_NEAR(expected_translation_link1.GetZ(), transform_from_urdf_link1.GetTranslation().GetZ(), 1e-5);

        const auto link2_pose = link2_ptr->SemanticPose();
        const AZ::Transform transform_from_urdf_link2 = ROS2::Utils::GetLocalTransformURDF(link2_pose);
        EXPECT_NEAR(expected_translation_link2.GetX(), transform_from_urdf_link2.GetTranslation().GetX(), 1e-5);
        EXPECT_NEAR(expected_translation_link2.GetY(), transform_from_urdf_link2.GetTranslation().GetY(), 1e-5);
        EXPECT_NEAR(expected_translation_link2.GetZ(), transform_from_urdf_link2.GetTranslation().GetZ(), 1e-5);

        const auto link3_pose = link3_ptr->SemanticPose();
        const AZ::Transform transform_from_urdf_link3 = ROS2::Utils::GetLocalTransformURDF(link3_pose);
        EXPECT_NEAR(expected_translation_link3.GetX(), transform_from_urdf_link3.GetTranslation().GetX(), 1e-5);
        EXPECT_NEAR(expected_translation_link3.GetY(), transform_from_urdf_link3.GetTranslation().GetY(), 1e-5);
        EXPECT_NEAR(expected_translation_link3.GetZ(), transform_from_urdf_link3.GetTranslation().GetZ(), 1e-5);
    }

    TEST_F(UrdfParserTest, TestQueryJointsForParentLink_Succeeds)
    {
        const auto xmlStr = GetURDFWithTransforms();
        sdf::ParserConfig parserConfig;
        const auto sdfRootOutcome = ROS2::UrdfParser::Parse(xmlStr, parserConfig);
        ASSERT_TRUE(sdfRootOutcome);
        const sdf::Root& sdfRoot = sdfRootOutcome.GetRoot();
        const sdf::Model* model = sdfRoot.Model();
        ASSERT_NE(nullptr, model);
        auto joints = ROS2::Utils::GetJointsForParentLink(*model, "base_link");
        EXPECT_EQ(1, joints.size());

        auto jointToNameProjection = [](const sdf::Joint* joint)
        {
            return AZStd::string_view(joint->Name().c_str(), joint->Name().size());
        };
        ASSERT_TRUE(AZStd::ranges::contains(joints, "joint0", jointToNameProjection));

        // Now check the middle link of "link2"
        joints = ROS2::Utils::GetJointsForParentLink(*model, "link2");
        EXPECT_EQ(1, joints.size());

        ASSERT_TRUE(AZStd::ranges::contains(joints, "joint1", jointToNameProjection));
    }

    TEST_F(UrdfParserTest, TestQueryJointsForChildLink_Succeeds)
    {
        const auto xmlStr = GetURDFWithTransforms();
        sdf::ParserConfig parserConfig;
        const auto sdfRootOutcome = ROS2::UrdfParser::Parse(xmlStr, parserConfig);
        ASSERT_TRUE(sdfRootOutcome);
        const sdf::Root& sdfRoot = sdfRootOutcome.GetRoot();
        const sdf::Model* model = sdfRoot.Model();
        ASSERT_NE(nullptr, model);
        auto joints = ROS2::Utils::GetJointsForChildLink(*model, "link2");
        EXPECT_EQ(1, joints.size());

        auto jointToNameProjection = [](const sdf::Joint* joint)
        {
            return AZStd::string_view(joint->Name().c_str(), joint->Name().size());
        };
        ASSERT_TRUE(AZStd::ranges::contains(joints, "joint0", jointToNameProjection));

        // Now check the final link of "link3"
        joints = ROS2::Utils::GetJointsForChildLink(*model, "link3");
        EXPECT_EQ(1, joints.size());

        ASSERT_TRUE(AZStd::ranges::contains(joints, "joint1", jointToNameProjection));
    }

    TEST_F(UrdfParserTest, TestPathResolve_ValidAbsolutePath_ResolvesCorrectly)
    {
        // Verify that an absolute path that wouldn't be resolved by prefixes or ancestor paths
        // or the AMENT_PREFIX_PATH will still resolve correctly as long as the absolute path exists
        // (as determined by the mocked-out FileExistsCallback below).
        constexpr AZ::IO::PathView dae = "file:///usr/ros/humble/meshes/bar.dae";
        constexpr AZ::IO::PathView urdf = "/home/foo/ros_ws/install/foo_robot/foo_robot.urdf";
        constexpr AZ::IO::PathView expectedResult = "/usr/ros/humble/meshes/bar.dae";
        auto result = ROS2::Utils::ResolveAssetPath(
            dae,
            urdf, "", GetTestSettings(),
            [expectedResult](const AZ::IO::PathView& p) -> bool
            {
                // Only a file name that matches the expected result will return that it exists.
                return p == expectedResult;
            });
        EXPECT_EQ(result, expectedResult);
    }

    TEST_F(UrdfParserTest, TestPathResolve_InvalidAbsolutePath_ReturnsEmptyPath)
    {
        // Verify that an absolute path that isn't found (as determined by the mocked-out
        // FileExistsCallback below) returns an empty path.
        constexpr AZ::IO::PathView dae = "file:///usr/ros/humble/meshes/bar.dae";
        constexpr AZ::IO::PathView urdf = "/home/foo/ros_ws/install/foo_robot/foo_robot.urdf";
        constexpr AZ::IO::PathView expectedResult = "";
        auto result = ROS2::Utils::ResolveAssetPath(
            dae,
            urdf, "", GetTestSettings(),
            [](const AZ::IO::PathView& p) -> bool
            {
                // Always return "not found" for all file names.
                return false;
            });
        EXPECT_EQ(result, expectedResult);
    }

    TEST_F(UrdfParserTest, TestPathResolve_ValidRelativePath_ResolvesCorrectly)
    {
        // Verify that a path that is intended to be relative to the location of the .urdf file resolves correctly.
        constexpr AZ::IO::PathView dae = "meshes/bar.dae";
        constexpr AZ::IO::PathView urdf = "/home/foo/ros_ws/install/foo_robot/foo_robot.urdf";
        constexpr AZ::IO::PathView expectedResult = "/home/foo/ros_ws/install/foo_robot/meshes/bar.dae";
        auto result = ROS2::Utils::ResolveAssetPath(
            dae,
            urdf, "", GetTestSettings(),
            [expectedResult](const AZ::IO::PathView& p) -> bool
            {
                // Only a file name that matches the expected result will return that it exists.
                return p == expectedResult;
            });
        EXPECT_EQ(result, expectedResult);
    }

    TEST_F(UrdfParserTest, TestPathResolve_InvalidRelativePath_ReturnsEmptyPath)
    {
        // Verify that a relative path that can't be found returns an empty path.
        constexpr AZ::IO::PathView dae = "meshes/bar.dae";
        constexpr AZ::IO::PathView urdf = "/home/foo/ros_ws/install/foo_robot/foo_robot.urdf";
        constexpr AZ::IO::PathView expectedResult = "";
        auto result = ROS2::Utils::ResolveAssetPath(
            dae,
            urdf, "", GetTestSettings(),
            [](const AZ::IO::PathView& p) -> bool
            {
                // Always return "not found"
                return false;
            });
        EXPECT_EQ(result, expectedResult);
    }

    TEST_F(UrdfParserTest, TestPathResolve_ValidAmentRelativePathButNoPrefix_ReturnsEmptyPath)
    {
        // Verify that a path that is intended to be relative to the location of one of the AMENT_PREFIX_PATH paths
        // doesn't resolve if it doesn't start with a prefix like "package://" or "model://".
        constexpr AZ::IO::PathView dae = "robot/meshes/bar.dae";
        constexpr AZ::IO::PathView urdf = "/home/foo/ros_ws/install/foo_robot/foo_robot.urdf";
        constexpr AZStd::string_view amentPrefixPath = "/ament/path1:/ament/path2";
        constexpr AZ::IO::PathView expectedResult = "";
        auto result = ROS2::Utils::ResolveAssetPath(
            dae,
            urdf, amentPrefixPath, GetTestSettings(),
            [](const AZ::IO::PathView& p) -> bool
            {
                // For an AMENT_PREFIX_PATH to be a valid match, the share/<package>/package.xml and share/<relative path>
                // both need to exist. We'll return that both exist, but since the dae file entry doesn't start with
                // "package://" or "model://", it shouldn't get resolved.
                return (p == AZ::IO::PathView("/ament/path2/share/robot/package.xml")) || (p == "/ament/path2/share/robot/meshes/bar.dae");
            });
        EXPECT_EQ(result, expectedResult);
    }

    TEST_F(UrdfParserTest, TestPathResolve_ValidAmentRelativePathAndPrefix_ResolvesCorrectly)
    {
        // Verify that a path that is intended to be relative to the location of one of the AMENT_PREFIX_PATH paths
        // doesn't resolve if it doesn't start with a prefix like "package://" or "model://".
        constexpr AZ::IO::PathView dae = "model://robot/meshes/bar.dae";
        constexpr AZ::IO::PathView urdf = "/home/foo/ros_ws/install/foo_robot/foo_robot.urdf";
        constexpr AZStd::string_view amentPrefixPath = "/ament/path1:/ament/path2";
        constexpr AZ::IO::PathView expectedResult = "/ament/path2/share/robot/meshes/bar.dae";
        auto result = ROS2::Utils::ResolveAssetPath(
            dae,
            urdf, amentPrefixPath, GetTestSettings(),
            [expectedResult](const AZ::IO::PathView& p) -> bool
            {
                // For an AMENT_PREFIX_PATH to be a valid match, the share/<package>/package.xml and share/<relative path>
                // both need to exist.
                return (p == AZ::IO::PathView("/ament/path2/share/robot/package.xml")) || (p == expectedResult);
            });
        EXPECT_EQ(result, expectedResult);
    }

    TEST_F(UrdfParserTest, TestPathResolve_ValidPathRelativeToAncestorPath_ResolvesCorrectly)
    {
        // Verify that a path that's relative to an ancestor path of the urdf file resolves correctly
        constexpr AZ::IO::PathView dae = "package://meshes/bar.dae";
        constexpr AZ::IO::PathView urdf = "/home/foo/ros_ws/install/foo_robot/description/foo_robot.urdf";
        constexpr AZ::IO::PathView expectedResult = "/home/foo/ros_ws/install/foo_robot/meshes/bar.dae";
        auto mockFileSystem = [&](const AZ::IO::PathView& p) -> bool
        {
            return p == expectedResult;
        };
        auto result = ROS2::Utils::ResolveAssetPath(dae, urdf, "", GetTestSettings(), mockFileSystem);
        EXPECT_EQ(result, expectedResult);
    }

    TEST_F(UrdfParserTest, TestPathResolve_ValidPathRelativeToAncestorPath_FailsToResolveWhenAncestorPathsDisabled)
    {
        // Verify that a path that's relative to an ancestor path of the urdf file fails to resolve if "use ancestor paths" is disabled.
        constexpr AZ::IO::PathView dae = "package://meshes/bar.dae";
        constexpr AZ::IO::PathView urdf = "/home/foo/ros_ws/install/foo_robot/description/foo_robot.urdf";
        constexpr AZ::IO::PathView resolvedDae = "/home/foo/ros_ws/install/foo_robot/meshes/bar.dae";

        auto settings = GetTestSettings();
        settings.m_resolverSettings.m_useAncestorPaths = false;

        auto mockFileSystem = [&](const AZ::IO::PathView& p) -> bool
        {
            // This should never return true, because this path should never get requested.
            return p == resolvedDae;
        };
        auto result = ROS2::Utils::ResolveAssetPath(dae, urdf, "", settings, mockFileSystem);
        EXPECT_EQ(result, "");
    }

    TEST_F(UrdfParserTest, TestPathResolvementExplicitPackageName)
    {
        constexpr AZ::IO::PathView dae = "package://foo_robot/meshes/bar.dae";
        constexpr AZ::IO::PathView urdf = "/home/foo/ros_ws/install/foo_robot/share/foo_robot/description/foo_robot.urdf";
        constexpr AZ::IO::PathView xml = "/home/foo/ros_ws/install/foo_robot/share/foo_robot/package.xml";
        constexpr AZ::IO::PathView resolvedDae = "/home/foo/ros_ws/install/foo_robot/share/foo_robot/meshes/bar.dae";
        auto mockFileSystem = [&](const AZ::IO::PathView& p) -> bool
        {
            return (p == xml) || (p == resolvedDae);
        };
        auto result = ROS2::Utils::ResolveAssetPath(dae, urdf, "/home/foo/ros_ws/install/foo_robot", GetTestSettings(), mockFileSystem);
        EXPECT_EQ(result, resolvedDae);
    }

    TEST_F(UrdfParserTest, ResolvePath_UsingModelUriScheme_Succeeds)
    {
        constexpr AZ::IO::PathView dae = "model://foo_robot/meshes/bar.dae";
        constexpr AZ::IO::PathView urdf = "/home/foo/ros_ws/install/foo_robot/share/foo_robot/description/foo_robot.urdf";
        constexpr AZ::IO::PathView xml = "/home/foo/ros_ws/install/foo_robot/share/foo_robot/package.xml";
        constexpr AZ::IO::PathView resolvedDae = "/home/foo/ros_ws/install/foo_robot/share/foo_robot/meshes/bar.dae";
        auto mockFileSystem = [&](const AZ::IO::PathView& p) -> bool
        {
            return (p == xml) || (p == resolvedDae);
        };
        auto result = ROS2::Utils::ResolveAssetPath(dae, urdf, "/home/foo/ros_ws/install/foo_robot", GetTestSettings(), mockFileSystem);
        EXPECT_EQ(result, resolvedDae);
    }

    TEST_F(UrdfParserTest, XacroParseArgsInvalid)
    {
        AZStd::string xacroParams = GetXacroParams();
        ROS2::Utils::xacro::Params params = ROS2::Utils::xacro::GetParameterFromXacroData("");
        EXPECT_EQ(params.size(), 0);
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
