/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <AzTest/AzTest.h>
#include <AzCore/UnitTest/TestTypes.h>
#include <Algorithms/LossFunctions.h>

namespace UnitTest
{
    class MachineLearning_LossFunctions
        : public UnitTest::LeakDetectionFixture
    {
    };

    TEST_F(MachineLearning_LossFunctions, TestMeanSquaredError)
    {
        AZ::VectorN expected = AZ::VectorN::CreateZero(1024);
        AZ::VectorN actual = AZ::VectorN::CreateOne(1024);

        const float totalLoss1 = MachineLearning::ComputeTotalCost(MachineLearning::LossFunctions::MeanSquaredError, expected, actual);
        EXPECT_EQ(totalLoss1, 1024.0f);
    }
}
