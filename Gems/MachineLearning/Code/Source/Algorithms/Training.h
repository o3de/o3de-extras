/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/Math/VectorN.h>
#include <MachineLearning/INeuralNetwork.h>
#include <MachineLearning/ILabeledTrainingData.h>

namespace MachineLearning
{
    //! Calculates the average cost of the provided model on the set of labeled test data using the requested loss function.
    float ComputeCurrentCost(INeuralNetworkPtr Model, ILabeledTrainingDataPtr TestData, LossFunctions CostFunction);

    //! Performs a supervised learning training cycle.
    //! Supervised learning is a form of machine learning where a model is provided a set of training data with expected output
    //! Training then takes place in an iterative loop where the total error (cost, loss) of the model is minimized
    //! This differs from unsupervised learning, where the training data lacks any form of labeling (expected correct output), and
    //! the model is expected to learn the underlying structures of data on its own.
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
    );
}
