/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <Nodes/SupervisedLearning.h>
#include <Models/MultilayerPerceptron.h>
#include <MachineLearning/ILabeledTrainingData.h>
#include <Algorithms/Training.h>
#include <AzCore/Console/ILogger.h>
#include <AzCore/std/chrono/chrono.h>

namespace MachineLearning
{
    INeuralNetworkPtr SupervisedLearning::In
    (
        INeuralNetworkPtr Model,
        ILabeledTrainingDataPtr TrainingData, 
        ILabeledTrainingDataPtr TestData,
        AZStd::size_t CostFunction,
        AZStd::size_t TotalIterations,
        AZStd::size_t BatchSize,
        float LearningRate,
        float LearningRateDecay,
        float EarlyStopCost
    )
    {
        SupervisedLearningCycle trainingInstance(Model, TrainingData, TestData, static_cast<LossFunctions>(CostFunction), TotalIterations, BatchSize, LearningRate, LearningRateDecay, EarlyStopCost);

        trainingInstance.StartTraining();
        while (!trainingInstance.m_trainingComplete)
        {
            AZStd::this_thread::sleep_for(AZStd::chrono::milliseconds(1));
        }
        trainingInstance.StopTraining();

        return Model;
    }
}
