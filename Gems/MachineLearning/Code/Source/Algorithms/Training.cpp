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
#include <AzCore/Jobs/JobCompletion.h>
#include <AzCore/Jobs/JobFunction.h>
#include <numeric>
#include <random>

namespace MachineLearning
{
    SupervisedLearningCycle::SupervisedLearningCycle()
    {
        AZ::JobManagerDesc jobDesc;
        jobDesc.m_jobManagerName = "MachineLearning Training";
        jobDesc.m_workerThreads.push_back(AZ::JobManagerThreadDesc()); // Just one thread
        m_trainingJobManager = AZStd::make_unique<AZ::JobManager>(jobDesc);
        m_trainingjobContext = AZStd::make_unique<AZ::JobContext>(*m_trainingJobManager);
    }

    SupervisedLearningCycle::SupervisedLearningCycle
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
    ) : SupervisedLearningCycle()
    {
        m_model = model;
        m_trainData = trainingData;
        m_testData = testData;
        m_costFunction = costFunction;
        m_totalIterations = totalIterations;
        m_batchSize = batchSize;
        m_learningRate = learningRate;
        m_learningRateDecay = learningRateDecay;
        m_earlyStopCost = earlyStopCost;
    }

    void SupervisedLearningCycle::InitializeContexts()
    {
        if (m_inferenceContext == nullptr)
        {
            m_inferenceContext.reset(m_model->CreateInferenceContext());
            m_trainingContext.reset(m_model->CreateTrainingContext());
        }
    }

    void SupervisedLearningCycle::StartTraining()
    {
        InitializeContexts();

        const AZStd::size_t totalTrainingSize = m_trainData.GetSampleCount();

        // Start training
        m_currentEpoch = 0;
        m_trainingComplete = false;
        m_currentIndex = 0;
        if (m_shuffleTrainingData)
        {
            m_trainData.ShuffleSamples();
        }

        auto job = [this]()
        {
            ExecTraining();
        };
        AZ::Job* trainingJob = AZ::CreateJobFunction(job, true, m_trainingjobContext.get());
        trainingJob->Start();
    }

    void SupervisedLearningCycle::StopTraining()
    {
        m_trainingComplete = true;
    }

    void SupervisedLearningCycle::ExecTraining()
    {
        const AZStd::size_t totalTrainingSize = m_trainData.GetSampleCount();
        while (!m_trainingComplete)
        {
            if (m_currentIndex >= totalTrainingSize)
            {
                // If we run out of training samples, we increment our epoch and reset for a new pass of the training data
                m_currentIndex = 0;
                m_learningRate *= m_learningRateDecay;

                // We reshuffle the training data indices each epoch to avoid patterns in the training data
                if (m_shuffleTrainingData)
                {
                    AZStd::lock_guard lock(m_mutex);
                    m_trainData.ShuffleSamples();
                }
                ++m_currentEpoch;

                // Generally we want to keep monitoring the model's performance on both test and training data
                // This allows us to detect if we're overfitting the model to the training data
                float currentTestCost = ComputeCurrentCost(m_testData, m_costFunction);
                float currentTrainCost = ComputeCurrentCost(m_trainData, m_costFunction);
                m_testCosts.PushBackItem(currentTestCost);
                m_trainCosts.PushBackItem(currentTrainCost);
                if ((currentTestCost < m_earlyStopCost) || (m_currentEpoch >= m_totalIterations))
                {
                    m_trainingComplete = true;
                    return;
                }
            }

            for (uint32_t batch = 0; (batch < m_batchSize) && (m_currentIndex < totalTrainingSize); ++batch, ++m_currentIndex)
            {
                const AZ::VectorN& activations = m_trainData.GetDataByIndex(m_currentIndex);
                const AZ::VectorN& label = m_trainData.GetLabelByIndex(m_currentIndex);
                m_model->Reverse(m_trainingContext.get(), m_costFunction, activations, label);
            }
            AZStd::lock_guard lock(m_mutex);
            m_model->GradientDescent(m_trainingContext.get(), m_learningRate);
        }
    }

    float SupervisedLearningCycle::ComputeCurrentCost(ILabeledTrainingData& testData, LossFunctions costFunction)
    {
        InitializeContexts();

        double result = 0.0;
        const AZStd::size_t totalTestSize = testData.GetSampleCount();
        for (uint32_t iter = 0; iter < totalTestSize; ++iter)
        {
            const AZ::VectorN& activations = testData.GetDataByIndex(iter);
            const AZ::VectorN& label = testData.GetLabelByIndex(iter);
            const AZ::VectorN* output = m_model->Forward(m_inferenceContext.get(), activations);
            result += static_cast<double>(ComputeTotalCost(costFunction, label, *output));
        }
        result /= static_cast<double>(totalTestSize);
        return static_cast<float>(result);
    }
}
