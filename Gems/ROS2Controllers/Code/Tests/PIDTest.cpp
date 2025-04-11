/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <AzCore/UnitTest/TestTypes.h>
#include <AzTest/AzTest.h>

#include <ROS2Controllers/Controllers/PidConfiguration.h>

namespace UnitTest
{
    static const uint64_t secToNanosec = 1e9;

    class PIDTest : public LeakDetectionFixture
    {
    };

    TEST_F(PIDTest, iClampAntiwindup)
    {
        double iGain = 1.0;
        double iMin = -1.0;
        double iMax = 1.0;

        ROS2::Controllers::PidConfiguration pid(0.0, iGain, 0.0, iMax, iMin, true, 1.0);
        pid.InitializePid();

        double output = 0.0;

        output = pid.ComputeCommand(-10.0, 1 * secToNanosec);
        EXPECT_EQ(0.0, output);

        output = pid.ComputeCommand(30.0, 1 * secToNanosec);
        EXPECT_EQ(1.0, output);
    }

    TEST_F(PIDTest, iClampNoGain)
    {
        double iGain = 0.0;
        double iMin = -1.0;
        double iMax = 1.0;

        ROS2::Controllers::PidConfiguration pid(0.0, iGain, 0.0, iMax, iMin, false, 0.0);
        pid.InitializePid();

        double output = 0.0;

        output = pid.ComputeCommand(-1.0, 1 * secToNanosec);
        EXPECT_LE(iMin, output);
        EXPECT_LE(output, iMax);
        EXPECT_EQ(0.0, output);

        output = pid.ComputeCommand(-1.0, 1 * secToNanosec);
        EXPECT_LE(iMin, output);
        EXPECT_LE(output, iMax);
        EXPECT_EQ(0.0, output);
    }

    TEST_F(PIDTest, iAntiwindup)
    {
        double iGain = 2.0;
        double iMin = -1.0;
        double iMax = 1.0;

        ROS2::Controllers::PidConfiguration pid(0.0, iGain, 0.0, iMax, iMin, true, 0.0);
        pid.InitializePid();

        double output = 0.0;

        output = pid.ComputeCommand(-1.0, 1 * secToNanosec);
        EXPECT_EQ(-1.0, output);

        output = pid.ComputeCommand(-1.0, 1 * secToNanosec);
        EXPECT_EQ(-1.0, output);

        output = pid.ComputeCommand(0.5, 1 * secToNanosec);
        EXPECT_EQ(0.0, output);

        output = pid.ComputeCommand(-1.0, 1 * secToNanosec);
        EXPECT_EQ(-1.0, output);
    }

    TEST_F(PIDTest, negativeIAntiwindup)
    {
        double iGain = -2.5;
        double iMin = -0.2;
        double iMax = 0.5;

        ROS2::Controllers::PidConfiguration pid(0.0, iGain, 0.0, iMax, iMin, true, 0.0);
        pid.InitializePid();

        double output = 0.0;

        output = pid.ComputeCommand(0.1, 1 * secToNanosec);
        EXPECT_EQ(-0.2, output);

        output = pid.ComputeCommand(0.1, 1 * secToNanosec);
        EXPECT_EQ(-0.2, output);

        output = pid.ComputeCommand(-0.05, 1 * secToNanosec);
        EXPECT_EQ(-0.075, output);

        output = pid.ComputeCommand(0.1, 1 * secToNanosec);
        EXPECT_EQ(-0.2, output);
    }

    TEST_F(PIDTest, pOnly)
    {
        ROS2::Controllers::PidConfiguration pid(1.0, 0.0, 0.0, 0.0, 0.0, false, 0.0);
        pid.InitializePid();

        double output = 0.0;

        output = pid.ComputeCommand(-0.5, 1 * secToNanosec);
        EXPECT_EQ(-0.5, output);

        output = pid.ComputeCommand(-0.5, 1 * secToNanosec);
        EXPECT_EQ(-0.5, output);

        output = pid.ComputeCommand(-1.0, 1 * secToNanosec);
        EXPECT_EQ(-1.0, output);

        output = pid.ComputeCommand(0.5, 1 * secToNanosec);
        EXPECT_EQ(0.5, output);
    }

    TEST_F(PIDTest, iOnly)
    {
        ROS2::Controllers::PidConfiguration pid(0.0, 1.0, 0.0, 5.0, -5.0, false, 0.0);
        pid.InitializePid();

        double output = 0.0;

        output = pid.ComputeCommand(-0.5, 1 * secToNanosec);
        EXPECT_EQ(-0.5, output);

        output = pid.ComputeCommand(-0.5, 1 * secToNanosec);
        EXPECT_EQ(-1.0, output);

        output = pid.ComputeCommand(0.0, 1 * secToNanosec);
        EXPECT_EQ(-1.0, output);

        output = pid.ComputeCommand(0.0, 1 * secToNanosec);
        EXPECT_EQ(-1.0, output);

        output = pid.ComputeCommand(1.0, 1 * secToNanosec);
        EXPECT_EQ(0.0, output);

        output = pid.ComputeCommand(-0.5, 1 * secToNanosec);
        EXPECT_EQ(-0.5, output);
    }

    TEST_F(PIDTest, dOnly)
    {
        ROS2::Controllers::PidConfiguration pid(0.0, 0.0, 1.0, 0.0, 0.0, false, 0.0);
        pid.InitializePid();

        double output = 0.0;

        output = pid.ComputeCommand(-0.5, 1 * secToNanosec);
        EXPECT_EQ(-0.5, output);

        output = pid.ComputeCommand(-0.5, 1 * secToNanosec);
        EXPECT_EQ(0.0, output);

        output = pid.ComputeCommand(-0.5, 0 * secToNanosec);
        EXPECT_EQ(0.0, output);

        output = pid.ComputeCommand(-1.0, 1 * secToNanosec);
        EXPECT_EQ(-0.5, output);

        output = pid.ComputeCommand(-0.5, 1 * secToNanosec);
        EXPECT_EQ(0.5, output);
    }

    TEST_F(PIDTest, completePID)
    {
        ROS2::Controllers::PidConfiguration pid(1.0, 1.0, 1.0, 5.0, -5.0, false, 0.0);
        pid.InitializePid();

        double output = 0.0;

        output = pid.ComputeCommand(-0.5, 1 * secToNanosec);
        EXPECT_EQ(-1.5, output);

        output = pid.ComputeCommand(-0.5, 1 * secToNanosec);
        EXPECT_EQ(-1.5, output);

        output = pid.ComputeCommand(-1.0, 1 * secToNanosec);
        EXPECT_EQ(-3.5, output);
    }
} // namespace UnitTest
