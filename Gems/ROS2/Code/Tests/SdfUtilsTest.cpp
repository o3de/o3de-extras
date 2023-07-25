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
#include <RobotImporter/SDFormat/Parser.h>
#include <RobotImporter/Utils/RobotImporterUtils.h>

#include "SdfModel.h"
#include <sdf/Box.hh>
#include <sdf/sdf.hh>

namespace UnitTest
{

    class SdfUtilsTest : public LeakDetectionFixture
    {
    };

    TEST_F(SdfUtilsTest, CheckSensorCount)
    {
        const auto xmlStr1 = SdfModel::GetSdfWithOneLink();
        const auto sdfRoot1 = ROS2::SDFormat::Parser::Parse(xmlStr1);

        const auto& allSensors1 = ROS2::Utils::SDFormat::GetAllSensors(sdfRoot1);
        EXPECT_EQ(allSensors1.size(), 1U);
        ASSERT_TRUE(allSensors1.contains("camera"));
        EXPECT_EQ(allSensors1.at("camera")->Name(), "camera");

        const auto xmlStr2 = SdfModel::GetSdfWithTwoLinksAndJoint();
        const auto sdfRoot2 = ROS2::SDFormat::Parser::Parse(xmlStr2);

        const auto& allSensors2 = ROS2::Utils::SDFormat::GetAllSensors(sdfRoot2);
        EXPECT_EQ(allSensors2.size(), 2U);
        ASSERT_TRUE(allSensors2.contains("camera"));
        ASSERT_TRUE(allSensors2.contains("laser"));
        EXPECT_EQ(allSensors2.at("camera")->Name(), "camera");
        EXPECT_EQ(allSensors2.at("laser")->Name(), "laser");
    }

    TEST_F(SdfUtilsTest, CheckPluginCount)
    {
        const auto xmlStr1 = SdfModel::GetSdfWithOneLink();
        const auto sdfRoot1 = ROS2::SDFormat::Parser::Parse(xmlStr1);

        const auto& allPlugins1 = ROS2::Utils::SDFormat::GetAllPlugins(sdfRoot1);
        ASSERT_TRUE(allPlugins1.empty());

        const auto xmlStr2 = SdfModel::GetSdfWithTwoLinksAndJoint();
        const auto sdfRoot2 = ROS2::SDFormat::Parser::Parse(xmlStr2);

        const auto& allPlugins2 = ROS2::Utils::SDFormat::GetAllPlugins(sdfRoot2);
        EXPECT_EQ(allPlugins2.size(), 2U);
        ASSERT_TRUE(allPlugins2.contains("laser_plug"));
        ASSERT_TRUE(allPlugins2.contains("joint_state"));
        EXPECT_EQ(allPlugins2.at("laser_plug")->Filename(), "librayplugin.so");
        EXPECT_EQ(allPlugins2.at("joint_state")->Filename(), "libgazebo_ros_joint_state_publisher.so");
    }

    TEST_F(SdfUtilsTest, SupportedOptionsCheck)
    {
        const auto xmlStr = SdfModel::GetSdfWithTwoLinksAndJoint();
        const auto sdfRoot = ROS2::SDFormat::Parser::Parse(xmlStr);

        const auto& allSensors = ROS2::Utils::SDFormat::GetAllSensors(sdfRoot);
        EXPECT_EQ(allSensors.size(), 2U);

        ASSERT_TRUE(allSensors.contains("camera"));
        const auto& cameraElement = allSensors.at("camera")->Element();
        // clang-format off
        const AZStd::unordered_set<AZStd::string> cameraSupportedOptions{ ">pose",
                                                                          ">update_rate",
                                                                          ">camera>horizontal_fov",
                                                                          ">camera>image>width",
                                                                          ">camera>image>height",
                                                                          ">camera>clip>near",
                                                                          ">camera>clip>far" };
        // clang-format on
        const auto& unsupportedCameraOptions = ROS2::Utils::SDFormat::GetUnsupportedOptions(cameraElement, cameraSupportedOptions);
        EXPECT_EQ(unsupportedCameraOptions.size(), 0U);

        ASSERT_TRUE(allSensors.contains("laser"));
        const auto& laserElement = allSensors.at("laser")->Element();
        const AZStd::unordered_set<AZStd::string> laserSupportedOptions{ ">pose",
                                                                         ">update_rate",
                                                                         ">ray>scan>horizontal>samples",
                                                                         ">ray>scan>horizontal>resolution",
                                                                         ">ray>scan>horizontal>min_angle",
                                                                         ">ray>scan>horizontal>max_angle",
                                                                         ">ray>range>min",
                                                                         ">ray>range>max",
                                                                         ">ray>range>resolution" };
        const auto& unsupportedLaserOptions = ROS2::Utils::SDFormat::GetUnsupportedOptions(laserElement, laserSupportedOptions);
        EXPECT_EQ(unsupportedLaserOptions.size(), 2U);
        EXPECT_EQ(unsupportedLaserOptions[0U], ">always_on");
        EXPECT_EQ(unsupportedLaserOptions[1U], ">visualize");

        const auto& allPlugins = ROS2::Utils::SDFormat::GetAllPlugins(sdfRoot);
        EXPECT_EQ(allPlugins.size(), 2U);

        ASSERT_TRUE(allPlugins.contains("laser_plug"));
        const auto& laserPluginElement = allPlugins.at("laser_plug")->Element();
        const AZStd::unordered_set<AZStd::string> noSupportedOptions{};
        const auto& unsupportedLaserPluginOptions = ROS2::Utils::SDFormat::GetUnsupportedOptions(laserPluginElement, noSupportedOptions);
        EXPECT_EQ(unsupportedLaserPluginOptions.size(), 0U);

        ASSERT_TRUE(allPlugins.contains("joint_state"));
        const auto& jointPluginElement = allPlugins.at("joint_state")->Element();
        const AZStd::unordered_set<AZStd::string> jointSupportedOptions{ ">update_rate", ">joint_name" };
        const auto& unsupportedJointPluginOptions = ROS2::Utils::SDFormat::GetUnsupportedOptions(jointPluginElement, jointSupportedOptions);
        EXPECT_EQ(unsupportedJointPluginOptions.size(), 1U);
        EXPECT_EQ(unsupportedJointPluginOptions[0U], ">ros>argument");
    }
} // namespace UnitTest
