/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <Algorithms/Training.h>
#include <Algorithms/LossFunctions.h>
#include <AzCore/Math/SimdMath.h>
#include <AzCore/Console/ILogger.h>
#include <numeric>
#include <random>

namespace MachineLearning
{
    float ComputeCurrentCost(INeuralNetworkPtr Model, ILabeledTrainingDataPtr TestData, LossFunctions CostFunction)
    {
        const AZStd::size_t totalTestSize = TestData->GetSampleCount();

        double result = 0.0;
        for (uint32_t iter = 0; iter < totalTestSize; ++iter)
        {
            const AZ::VectorN& activations = TestData->GetDataByIndex(iter);
            const AZ::VectorN& label = TestData->GetLabelByIndex(iter);
            const AZ::VectorN* output = Model->Forward(activations);
            result += static_cast<double>(ComputeTotalCost(CostFunction, label, *output));
        }
        result /= static_cast<double>(totalTestSize);
        return static_cast<float>(result);
    }

    void SupervisedLearningCycle
    (
        INeuralNetworkPtr model,
        ILabeledTrainingDataPtr trainingData,
        ILabeledTrainingDataPtr testData,
        LossFunctions costFunction, 
        AZStd::size_t totalIterations,
        AZStd::size_t batchSize,
        float learningRate,
        float learningRateDecay,
        float earlyStopCost
    )
    {
        const AZStd::size_t totalTrainingSize = trainingData->GetSampleCount();
        const float initialCost = ComputeCurrentCost(model, testData, costFunction);
        AZLOG_INFO("Initial model cost prior to training: %f", initialCost);

        // Generate a set of training indices that we can later shuffle
        AZStd::vector<AZStd::size_t> indices;
        indices.resize(totalTrainingSize);
        std::iota(indices.begin(), indices.end(), 0);

        for (uint32_t epoch = 0; epoch < totalIterations; ++epoch)
        {
            // We reshuffle the training data indices each epoch to avoid patterns in the training data
            std::shuffle(indices.begin(), indices.end(), std::mt19937(std::random_device{}()));
            AZStd::size_t sampleCount = 0;
            for (uint32_t batch = 0; (batch < batchSize) && (sampleCount < totalTrainingSize); ++batch, ++sampleCount)
            {
                const AZ::VectorN& activations = trainingData->GetDataByIndex(indices[sampleCount]);
                const AZ::VectorN& label = trainingData->GetLabelByIndex(indices[sampleCount]);
                model->Reverse(costFunction, activations, label);
            }
            model->GradientDescent(learningRate);

            const float currentTestCost = ComputeCurrentCost(model, testData, costFunction);
            const float currentTrainCost = ComputeCurrentCost(model, trainingData, costFunction);
            AZLOG_INFO("Epoch %u, Test cost: %f, Train cost: %f, Learning rate: %f", epoch, currentTestCost, currentTrainCost, learningRate);
            if (currentTestCost < earlyStopCost)
            {
                AZLOG_INFO("Early stop threshold reached, exiting training loop: %f, %f", currentTestCost, earlyStopCost);
                break;
            }

            learningRate *= learningRateDecay;
        }
    }
}
