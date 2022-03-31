/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <UrdfParser.h>

#include <AzTest/AzTest.h>

namespace {

class UrdfParserTest : public ::testing::Test
{
    public:
        std::string GetUrdfWithOneLink()
        {
            std::string xmlStr =
            "<robot name=\"test\">"
            "  <link name=\"link1\">"
            "    <inertial>"
            "      <mass value=\"1.0\"/>"
            "      <inertia ixx=\"1.0\" iyy=\"1.0\" izz=\"1.0\" ixy=\"0\" ixz=\"0\" iyz=\"0\"/>"
            "    </inertial>"
            "    <visual>"
            "      <geometry>"
            "        <box size=\"1.0 2.0 1.0\"/>"
            "      </geometry>"
            "    </visual>"
            "    <collision>"
            "      <geometry>"
            "        <box size=\"1.0 2.0 1.0\"/>"
            "      </geometry>"
            "    </collision>"
            "  </link>"
            "</robot>";

            return xmlStr;
        }
    protected:
        ROS2::UrdfParser parser; 
};

TEST_F(UrdfParserTest, ParseUrdfWithOneLink)
{
    const auto xmlStr = GetUrdfWithOneLink();
    const auto urdf = parser.Parse(xmlStr);

    EXPECT_EQ(urdf->getName(), "test");

    std::vector<urdf::LinkSharedPtr> links;
    urdf->getLinks(links);
    EXPECT_EQ(links.size(), 1U);

    const auto link1 = urdf->getLink("link1");

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

} // namespace
