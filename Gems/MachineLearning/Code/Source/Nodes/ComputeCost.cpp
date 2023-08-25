/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <Nodes/ComputeCost.h>
#include <Models/MultilayerPerceptron.h>
#include <Algorithms/LossFunctions.h>

namespace MachineLearning
{
    float ComputeCost::In(INeuralNetworkPtr Model, LossFunctions LossFunction, AZ::VectorN Activations, AZ::VectorN ExpectedOutput)
    {
        AZStd::unique_ptr<IInferenceContext> inferenceContext;
        inferenceContext.reset(Model->CreateInferenceContext());
        const AZ::VectorN* modelOutput = Model->Forward(inferenceContext.get(), Activations);
        return ComputeTotalCost(LossFunction, ExpectedOutput, *modelOutput);
    }
}
