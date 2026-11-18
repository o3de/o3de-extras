/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <ROS2/Utilities/ROS2Conversions.h>
#include <gtest/gtest.h>

namespace UnitTest
{
    namespace
    {
        builtin_interfaces::msg::Time MakeTime(int32_t sec, uint32_t nanosec)
        {
            builtin_interfaces::msg::Time time;
            time.sec = sec;
            time.nanosec = nanosec;
            return time;
        }
    } // namespace

    TEST(ROS2ConversionsTest, TimeDifferenceWithinOneSecond)
    {
        EXPECT_NEAR(ROS2::ROS2Conversions::GetTimeDifference(MakeTime(100, 250'000'000), MakeTime(100, 750'000'000)), 0.5f, 1e-6f);
    }

    TEST(ROS2ConversionsTest, TimeDifferenceAcrossAWholeSecond)
    {
        // The nanoseconds of the end are smaller than those of the start: an unsigned subtraction wraps by about 4.29 s.
        EXPECT_NEAR(
            ROS2::ROS2Conversions::GetTimeDifference(MakeTime(1'794'950'000, 995'000'000), MakeTime(1'794'950'001, 1'000'000)),
            0.006f,
            1e-6f);
    }

    TEST(ROS2ConversionsTest, TimeDifferenceOverSeveralSeconds)
    {
        EXPECT_NEAR(ROS2::ROS2Conversions::GetTimeDifference(MakeTime(10, 900'000'000), MakeTime(13, 400'000'000)), 2.5f, 1e-6f);
    }

    TEST(ROS2ConversionsTest, NegativeTimeDifference)
    {
        EXPECT_NEAR(ROS2::ROS2Conversions::GetTimeDifference(MakeTime(101, 1'000'000), MakeTime(100, 995'000'000)), -0.006f, 1e-6f);
    }
} // namespace UnitTest
