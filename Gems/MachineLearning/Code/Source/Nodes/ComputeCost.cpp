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
    float ComputeCost::In(MachineLearning::INeuralNetworkPtr Model, MachineLearning::LossFunctions LossFunction, AZ::VectorN Activations, AZ::VectorN ExpectedOutput)
    {
        const AZ::VectorN& modelOutput = Model->Forward(Activations);
        return ComputeTotalCost(LossFunction, ExpectedOutput, modelOutput);
    }
}
