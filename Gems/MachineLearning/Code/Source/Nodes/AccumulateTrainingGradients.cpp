/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <Nodes/AccumulateTrainingGradients.h>
#include <Models/MultilayerPerceptron.h>

namespace MachineLearning
{
    INeuralNetworkPtr AccumulateTrainingGradients::In(INeuralNetworkPtr Model, LossFunctions LossFunction, AZ::VectorN Activations, AZ::VectorN ExpectedOutput)
    {
        Model->Reverse(LossFunction, Activations, ExpectedOutput);
        return Model;
    }
}
