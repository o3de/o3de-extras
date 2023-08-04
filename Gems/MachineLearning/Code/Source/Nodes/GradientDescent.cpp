/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <Nodes/GradientDescent.h>
#include <Models/MultilayerPerceptron.h>

namespace MachineLearning
{
    MachineLearning::INeuralNetworkPtr GradientDescent::In(MachineLearning::INeuralNetworkPtr Model, float LearningRate)
    {
        Model->GradientDescent(LearningRate);
        return Model;
    }
}
