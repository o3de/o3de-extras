/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <Nodes/FeedForward.h>
#include <Models/MultilayerPerceptron.h>

namespace MachineLearning
{
    AZ::VectorN FeedForward::In(MachineLearning::INeuralNetworkPtr Model, AZ::VectorN Activations)
    {
        AZ::VectorN results = Model->Forward(Activations);
        return results;
    }
}
