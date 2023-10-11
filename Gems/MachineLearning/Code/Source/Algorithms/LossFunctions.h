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

namespace MachineLearning
{
    //! This is a useful helper that simply computes the total cost provided a loss function, and expected and actual outputs.
    float ComputeTotalCost(LossFunctions lossFunction, const AZ::VectorN& expected, const AZ::VectorN& actual);

    //! Computes the gradient of the loss using across all elements of the source vectors using the requested cost function.
    void ComputeLoss(LossFunctions lossFunction, const AZ::VectorN& expected, const AZ::VectorN& actual, AZ::VectorN& output);

    //! Computes the derivative of the rectified linear unit function (ReLU) applied to all elements of the source vector.
    void MeanSquaredError(const AZ::VectorN& expected, const AZ::VectorN& actual, AZ::VectorN& output);

    //! Computes the gradient of the loss using across all elements of the source vectors using the requested cost function.
    void ComputeLoss_Derivative(LossFunctions lossFunction, const AZ::VectorN& expected, const AZ::VectorN& actual, AZ::VectorN& output);

    //! Computes the derivative of the rectified linear unit function (ReLU) applied to all elements of the source vector.
    void MeanSquaredError_Derivative(const AZ::VectorN& expected, const AZ::VectorN& actual, AZ::VectorN& output);
}
