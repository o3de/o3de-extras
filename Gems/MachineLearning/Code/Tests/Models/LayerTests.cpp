/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <AzTest/AzTest.h>
#include <AzCore/UnitTest/TestTypes.h>
#include <Models/Layer.h>

namespace UnitTest
{
    class MachineLearning_Layers
        : public UnitTest::LeakDetectionFixture
    {
    };

    TEST_F(MachineLearning_Layers, TestConstructor)
    {
        // Construct a layer that takes 8 inputs and generates 4 outputs
        MachineLearning::Layer testLayer(MachineLearning::ActivationFunctions::Linear, 8, 4);
        EXPECT_EQ(testLayer.m_inputSize, 8);
        EXPECT_EQ(testLayer.m_outputSize, 4);
        EXPECT_EQ(testLayer.m_weights.GetColumnCount(), 8);
        EXPECT_EQ(testLayer.m_weights.GetRowCount(), 4);
        EXPECT_EQ(testLayer.m_biases.GetDimensionality(), 4);
        EXPECT_EQ(testLayer.m_output.GetDimensionality(), 4);
    }

    TEST_F(MachineLearning_Layers, TestForward)
    {
        // Construct a layer that takes 8 inputs and generates 4 outputs
        MachineLearning::Layer testLayer(MachineLearning::ActivationFunctions::Linear, 8, 4);
        testLayer.m_biases = AZ::VectorN::CreateOne(testLayer.m_biases.GetDimensionality());
        testLayer.m_weights = AZ::MatrixMxN::CreateZero(testLayer.m_weights.GetRowCount(), testLayer.m_weights.GetColumnCount());
        testLayer.m_weights += 1.0f;

        const AZ::VectorN ones = AZ::VectorN::CreateOne(8); // Input of all ones
        testLayer.Forward(ones);
        for (AZStd::size_t iter = 0; iter < testLayer.m_output.GetDimensionality(); ++iter)
        {
            ASSERT_FLOAT_EQ(testLayer.m_output.GetElement(iter), 9.0f); // 8 edges of 1's + 1 for the bias
        }

        const AZ::VectorN zeros = AZ::VectorN::CreateZero(8); // Input of all zeros
        testLayer.Forward(zeros);
        for (AZStd::size_t iter = 0; iter < testLayer.m_output.GetDimensionality(); ++iter)
        {
            ASSERT_FLOAT_EQ(testLayer.m_output.GetElement(iter), 1.0f); // Weights are all zero, leaving only the layer biases which are all set to 1
        }
    }
}
