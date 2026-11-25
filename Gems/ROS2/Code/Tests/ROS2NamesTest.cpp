/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <AzCore/std/utils.h>
#include <ROS2/ROS2NamesBus.h>
#include <gtest/gtest.h>

// The ROS2NamesRequestBus is handled by the ROS2 system component, which is activated by the test environment.
namespace UnitTest
{
    namespace
    {
        AZStd::string RosifyName(const AZStd::string& input)
        {
            AZStd::string rosified;
            ROS2::ROS2NamesRequestBus::BroadcastResult(rosified, &ROS2::ROS2NamesRequests::RosifyName, input);
            return rosified;
        }

        bool IsValidNameToken(const AZStd::string& token)
        {
            AZ::Outcome<void, AZStd::string> result = AZ::Failure(AZStd::string("ROS2NamesRequestBus is not connected"));
            ROS2::ROS2NamesRequestBus::BroadcastResult(result, &ROS2::ROS2NamesRequests::ValidateNameToken, token);
            return result.IsSuccess();
        }
    } // namespace

    TEST(ROS2NamesTest, RosifyName)
    {
        const AZStd::pair<const char*, const char*> cases[] = {
            { "robot_1", "robot_1" }, { "robot.1", "robot_1" }, { "robot/1", "robot_1" }, { "robot|1", "robot_1" },
            { "1robot", "o3de_1robot" }, { ".robot", "o3de__robot" }, { "_robot", "_robot" },
        };
        for (const auto& [input, expected] : cases)
        {
            EXPECT_EQ(RosifyName(input), expected) << input;
            EXPECT_TRUE(IsValidNameToken(RosifyName(input))) << input;
        }
    }

    TEST(ROS2NamesTest, ValidateNameToken)
    {
        for (const char* token : { "robot", "Robot_1", "_robot" })
        {
            EXPECT_TRUE(IsValidNameToken(token)) << token;
        }
        for (const char* token : { "", "1robot", "robot/1", "~robot", "{robot}", "robot.1", "robot|1", "robot 1" })
        {
            EXPECT_FALSE(IsValidNameToken(token)) << token;
        }
    }
} // namespace UnitTest
