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

    TEST_F(MachineLearning_Activations, OneHotArgMax)
    {
        AZStd::size_t testValue = 1;
        AZ::VectorN testVector;

        MachineLearning::OneHotEncode(testValue, 10, testVector);
        EXPECT_FLOAT_EQ(testVector.GetElement(0), 0.0f);
        EXPECT_FLOAT_EQ(testVector.GetElement(1), 1.0f);
        EXPECT_EQ(MachineLearning::ArgMaxDecode(testVector), testValue);

        testValue = 3;
        MachineLearning::OneHotEncode(testValue, 10, testVector);
        EXPECT_EQ(MachineLearning::ArgMaxDecode(testVector), testValue);

        testValue = 7;
        MachineLearning::OneHotEncode(testValue, 10, testVector);
        EXPECT_EQ(MachineLearning::ArgMaxDecode(testVector), testValue);

        testValue = 8;
        MachineLearning::OneHotEncode(testValue, 10, testVector);
        EXPECT_EQ(MachineLearning::ArgMaxDecode(testVector), testValue);
    }

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

    TEST_F(MachineLearning_Activations, TestSigmoid)
    {
        AZ::VectorN output = AZ::VectorN::CreateZero(1024);
        AZ::VectorN sourceVector = AZ::VectorN::CreateRandom(1024);
        sourceVector *= 100.0f;
        sourceVector -= 50.0f;
        MachineLearning::Sigmoid(sourceVector, output);

        // Sigmoid guarantees all outputs get squished between 0 and 1
        for (AZStd::size_t iter = 0; iter < output.GetDimensionality(); ++iter)
        {
            ASSERT_GE(output.GetElement(iter), 0.0f);
            ASSERT_LE(output.GetElement(iter), 1.0f);
        }
    }

    TEST_F(MachineLearning_Activations, TestSoftmax)
    {
        AZ::VectorN output = AZ::VectorN::CreateZero(1024);
        AZ::VectorN sourceVector = AZ::VectorN::CreateRandom(1024);
        sourceVector *= 100.0f;
        sourceVector -= 50.0f;
        MachineLearning::Softmax(sourceVector, output);

        // Sigmoid guarantees all outputs get squished between 0 and 1
        for (AZStd::size_t iter = 0; iter < output.GetDimensionality(); ++iter)
        {
            ASSERT_GE(output.GetElement(iter), 0.0f);
            ASSERT_LE(output.GetElement(iter), 1.0f);
        }

        // Additionally, the sum of all the elements should be <= 1, as softmax returns a probability distribution
        const float totalSum = output.L1Norm();
        // Between floating point precision and the estimates we use for exp(x), the total sum probability can be slightly greater than one
        // We add a small epsilon to account for this error
        ASSERT_LE(totalSum, 1.0f + AZ::Constants::Tolerance);
    }

    TEST_F(MachineLearning_Activations, TestLinear)
    {
        AZ::VectorN output = AZ::VectorN::CreateZero(1024);
        AZ::VectorN sourceVector = AZ::VectorN::CreateRandom(1024);
        sourceVector *= 100.0f;
        sourceVector -= 50.0f;
        MachineLearning::Linear(sourceVector, output);

        // Linear just returns the input provided
        // This makes it not suitable for anything with more than one layer
        for (AZStd::size_t iter = 0; iter < output.GetDimensionality(); ++iter)
        {
            ASSERT_EQ(output.GetElement(iter), sourceVector.GetElement(iter));
        }
    }
}
