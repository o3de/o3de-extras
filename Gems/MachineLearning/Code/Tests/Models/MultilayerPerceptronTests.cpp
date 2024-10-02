/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <AzTest/AzTest.h>
#include <AzCore/UnitTest/TestTypes.h>
#include <Models/MultilayerPerceptron.h>
#include <Algorithms/LossFunctions.h>

namespace UnitTest
{
    class MachineLearning_MLP
        : public UnitTest::LeakDetectionFixture
    {
    };

    TEST_F(MachineLearning_MLP, TestGradientCalculations)
    {
        // As the computations performed during gradient descent are non-trivial, this unit test carefully replicates the backward propagation example as laid out in this article
        // https://mattmazur.com/2015/03/17/a-step-by-step-backpropagation-example/
        // The example is extremely simple, so in the case of a unit test failure, this allows the maintainer to carefully trace unit test execution and compare its output with the article
        const float layer0Weights[] =
        {
            0.15f, 0.20f, 0.25f, 0.30f
        };
        const float layer0Biases[] = { 0.35f, 0.35f };

        const float layer1Weights[] =
        {
            0.40f, 0.45f, 0.50f, 0.55f
        };
        const float layer1Biases[] = { 0.60f, 0.60f };

        MachineLearning::MultilayerPerceptron mlp(2);
        MachineLearning::MlpInferenceContext inferenceData;
        MachineLearning::MlpTrainingContext trainingData;
        mlp.AddLayer(2, MachineLearning::ActivationFunctions::Sigmoid);
        mlp.AddLayer(2, MachineLearning::ActivationFunctions::Sigmoid);

        MachineLearning::Layer* layer0 = mlp.GetLayer(0);
        layer0->m_weights = AZ::MatrixMxN::CreateFromPackedFloats(2, 2, layer0Weights);
        layer0->m_biases = AZ::VectorN::CreateFromFloats(2, layer0Biases);

        MachineLearning::Layer* layer1 = mlp.GetLayer(1);
        layer1->m_weights = AZ::MatrixMxN::CreateFromPackedFloats(2, 2, layer1Weights);
        layer1->m_biases = AZ::VectorN::CreateFromFloats(2, layer1Biases);

        const float activations[] = { 0.05f, 0.10f };
        const float labels[] = { 0.01f, 0.99f };

        const AZ::VectorN trainingInput = AZ::VectorN::CreateFromFloats(2, activations);
        const AZ::VectorN trainingOutput = AZ::VectorN::CreateFromFloats(2, labels);

        const AZ::VectorN* actualOutput = mlp.Forward(&inferenceData, trainingInput);

        // Validate intermediate layer output given the initial weights and biases
        EXPECT_TRUE(AZ::IsCloseMag(inferenceData.m_layerData[0].m_output.GetElement(0), 0.5933f, 0.01f));
        EXPECT_TRUE(AZ::IsCloseMag(inferenceData.m_layerData[0].m_output.GetElement(1), 0.5969f, 0.01f));

        // Validate final model output given the initial weights and biases
        EXPECT_TRUE(AZ::IsCloseMag(actualOutput->GetElement(0), 0.75f, 0.01f));
        EXPECT_TRUE(AZ::IsCloseMag(actualOutput->GetElement(1), 0.77f, 0.01f));

        float cost = MachineLearning::ComputeTotalCost(MachineLearning::LossFunctions::MeanSquaredError, trainingOutput, *actualOutput);
        EXPECT_TRUE(AZ::IsCloseMag(cost, 0.60f, 0.01f));

        mlp.Reverse(&trainingData, MachineLearning::LossFunctions::MeanSquaredError, trainingInput, trainingOutput);

        // Check the activation gradients
        EXPECT_NEAR(trainingData.m_layerData[1].m_activationGradients.GetElement(0),  0.1385f, 0.01f);
        EXPECT_NEAR(trainingData.m_layerData[1].m_activationGradients.GetElement(1), -0.0381f, 0.01f);

        EXPECT_NEAR(trainingData.m_layerData[1].m_weightGradients.GetElement(0, 0),  0.0822f, 0.01f);
        EXPECT_NEAR(trainingData.m_layerData[1].m_weightGradients.GetElement(0, 1),  0.0826f, 0.01f);
        EXPECT_NEAR(trainingData.m_layerData[1].m_weightGradients.GetElement(1, 0), -0.0226f, 0.01f);
        EXPECT_NEAR(trainingData.m_layerData[1].m_weightGradients.GetElement(1, 1), -0.0227f, 0.01f);

        EXPECT_NEAR(trainingData.m_layerData[1].m_backpropagationGradients.GetElement(0), 0.0364f, 0.01f);
        EXPECT_NEAR(trainingData.m_layerData[1].m_backpropagationGradients.GetElement(1), 0.0414f, 0.01f);

        EXPECT_NEAR(trainingData.m_layerData[0].m_weightGradients.GetElement(0, 0),  0.0004f, 0.01f);
        EXPECT_NEAR(trainingData.m_layerData[0].m_weightGradients.GetElement(0, 1),  0.0008f, 0.01f);

        mlp.GradientDescent(&trainingData, 0.5f);

        EXPECT_NEAR(layer1->m_weights.GetElement(0, 0), 0.3590f, 0.01f);
        EXPECT_NEAR(layer1->m_weights.GetElement(0, 1), 0.4087f, 0.01f);
        EXPECT_NEAR(layer1->m_weights.GetElement(1, 0), 0.5113f, 0.01f);
        EXPECT_NEAR(layer1->m_weights.GetElement(1, 1), 0.5614f, 0.01f);

        EXPECT_NEAR(layer0->m_weights.GetElement(0, 0), 0.1498f, 0.01f);
        EXPECT_NEAR(layer0->m_weights.GetElement(0, 1), 0.1996f, 0.01f);
        EXPECT_NEAR(layer0->m_weights.GetElement(1, 0), 0.2495f, 0.01f);
        EXPECT_NEAR(layer0->m_weights.GetElement(1, 1), 0.2995f, 0.01f);

        // Now lets evaluate a whole training cycle
        const AZStd::size_t numTrainingLoops = 10000;
        for (AZStd::size_t iter = 0; iter < numTrainingLoops; ++iter)
        {
            mlp.Reverse(&trainingData, MachineLearning::LossFunctions::MeanSquaredError, trainingInput, trainingOutput);
            mlp.GradientDescent(&trainingData, 0.5f);
        }

        // We expect the total cost of the network on the training sample to be much lower after training
        const AZ::VectorN* trainedOutput = mlp.Forward(&inferenceData, trainingInput);
        float trainedCost = MachineLearning::ComputeTotalCost(MachineLearning::LossFunctions::MeanSquaredError, trainingOutput, *trainedOutput);
        EXPECT_LT(trainedCost, 5.0e-6f);
    }
}
