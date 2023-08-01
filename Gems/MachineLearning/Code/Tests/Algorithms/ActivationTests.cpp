/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <AzTest/AzTest.h>
#include <AzCore/UnitTest/TestTypes.h>
#include <Algorithms/Activations.h>

namespace UnitTest
{
    class MachineLearning_Activations
        : public UnitTest::LeakDetectionFixture
    {
    };

    TEST_F(MachineLearning_Activations, TestRelu)
    {
        AZ::VectorN output = AZ::VectorN::CreateZero(1024);
        AZ::VectorN sourceVector = AZ::VectorN::CreateRandom(1024);
        sourceVector *= 100.0f;
        sourceVector -= 50.0f;
        MachineLearning::ReLU(sourceVector, output);
    
        for (AZStd::size_t iter = 0; iter < output.GetDimensionality(); ++iter)
        {
            ASSERT_GE(output.GetElement(iter), 0.0f);
        }
    }
}
