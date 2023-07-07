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
        MachineLearning::Layer testLayer(8, 4);
        EXPECT_EQ(testLayer.m_inputSize, 8);
        EXPECT_EQ(testLayer.m_outputSize, 4);
        EXPECT_EQ(testLayer.m_weights.GetColumnCount(), 8);
        EXPECT_EQ(testLayer.m_weights.GetRowCount(), 4);
        EXPECT_EQ(testLayer.m_biases.GetDimensionality(), 4);
        EXPECT_EQ(testLayer.m_output.GetDimensionality(), 4);
    }
}
