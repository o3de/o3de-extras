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
    //! Reflection helper function
    AZStd::vector<AZ::Edit::EnumConstant<ActivationFunctions>> GetActivationEnumValues();

    //! One-hot encodes the provided value into the resulting vector output, which will have dimensionality maxValue.
    void OneHotEncode(AZStd::size_t value, AZStd::size_t maxValue, AZ::VectorN& output);

    //! Computes the requested activation function applied to all elements of the source vector.
    void Activate(ActivationFunctions activationFunction, const AZ::VectorN& sourceVector, AZ::VectorN& output);

    //! Computes the rectified linear unit function (ReLU) applied to all elements of the source vector.
    void ReLU(const AZ::VectorN& sourceVector, AZ::VectorN& output);

    //! Computes the sigmoid applied to all elements of the source vector.
    void Sigmoid(const AZ::VectorN& sourceVector, AZ::VectorN& output);

    //! Computes the softmax applied to all elements of the source vector.
    void Softmax(const AZ::VectorN& sourceVector, AZ::VectorN& output);

    //! Computes the linear activation function applied to all elements of the source vector.
    void Linear(const AZ::VectorN& sourceVector, AZ::VectorN& output);

    //! Computes the derivative of the requested activation function applied to all elements provided vector.
    //! The activationOutput input here is simply the output of calling Activate on the original source vector.
    void Activate_Derivative(ActivationFunctions activationFunction, const AZ::VectorN& activationOutput, const AZ::VectorN& backGradients, AZ::VectorN& output);

    //! Computes the derivative of the rectified linear unit function (ReLU) applied to all elements of the original source vector.
    void ReLU_Derivative(const AZ::VectorN& activationOutput, const AZ::VectorN& backGradients, AZ::VectorN& output);

    //! Computes the derivative of the sigmoid activation function applied to all elements of the original source vector.
    void Sigmoid_Derivative(const AZ::VectorN& activationOutput, const AZ::VectorN& backGradients, AZ::VectorN& output);

    //! Computes the derivative of the sigmoid activation function applied to all elements of the original source vector.
    void Softmax_Derivative(const AZ::VectorN& activationOutput, const AZ::VectorN& backGradients, AZ::VectorN& output);

    //! Computes the derivative linear activation function applied to all elements of the original source vector.
    void Linear_Derivative(const AZ::VectorN& activationOutput, const AZ::VectorN& backGradients, AZ::VectorN& output);
}
