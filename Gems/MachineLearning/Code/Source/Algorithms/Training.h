/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/Math/VectorN.h>
#include <AzCore/Jobs/JobManager.h>
#include <AzCore/Jobs/JobContext.h>
#include <AzCore/Threading/ThreadSafeDeque.h>
#include <MachineLearning/INeuralNetwork.h>
#include <Assets/TrainingDataView.h>

namespace MachineLearning
{
    //! Performs a supervised learning training cycle.
    //! Supervised learning is a form of machine learning where a model is provided a set of training data with expected output
    //! Training then takes place in an iterative loop where the total error (cost, loss) of the model is minimized
    //! This differs from unsupervised learning, where the training data lacks any form of labeling (expected correct output), and
    //! the model is expected to learn the underlying structures of data on its own.
    class SupervisedLearningCycle
    {
    public:

        SupervisedLearningCycle();

        SupervisedLearningCycle
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

        void InitializeContexts();

        void StartTraining();
        void StopTraining();

        AZStd::atomic<AZStd::size_t> m_currentEpoch = 0;
        std::atomic<bool> m_trainingComplete = true;

        AZ::ThreadSafeDeque<float> m_testCosts;
        AZ::ThreadSafeDeque<float> m_trainCosts;

        INeuralNetworkPtr m_model;
        bool m_shuffleTrainingData = true;
        TrainingDataView m_trainData;
        TrainingDataView m_testData;
        LossFunctions m_costFunction = LossFunctions::MeanSquaredError;
        AZStd::size_t m_totalIterations = 0;
        AZStd::size_t m_batchSize = 0;
        float m_learningRate = 0.0f;
        float m_learningRateDecay = 0.0f;
        float m_earlyStopCost = 0.0f;
        AZStd::size_t m_currentIndex = 0;
        AZStd::unique_ptr<IInferenceContext> m_inferenceContext;
        AZStd::unique_ptr<ITrainingContext> m_trainingContext;

    private:

        //! Calculates the average cost of the provided model on the set of labeled test data using the requested loss function.
        float ComputeCurrentCost(ILabeledTrainingData& testData, LossFunctions costFunction);
        void ExecTraining();

        AZStd::unique_ptr<AZ::JobManager> m_trainingJobManager;
        AZStd::unique_ptr<AZ::JobContext> m_trainingjobContext;

        //! Guards model state.
        mutable AZStd::recursive_mutex m_mutex;
    };
}
