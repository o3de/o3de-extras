/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/Math/VectorN.h>
#include <MachineLearning/Types.h>

namespace MachineLearning
{
    class INeuralNetwork
    {
    public:

        AZ_TYPE_INFO(INeuralNetwork, "{64E5B5B1-4A7D-489D-9A29-D9510BB7E17A}");

        virtual ~INeuralNetwork() = default;

        //! Adds a new layer to the network.
        virtual void AddLayer(AZStd::size_t layerDimensionality, ActivationFunctions activationFunction = ActivationFunctions::Linear) = 0;

        //! Returns the total number of layers in the network.
        virtual AZStd::size_t GetLayerCount() const = 0;

        //! Retrieves a specific layer from the network indexed by the layerIndex.
        virtual Layer& GetLayer(AZStd::size_t layerIndex) = 0;

        //! Returns the total number of parameters in the neural network.
        virtual AZStd::size_t GetParameterCount() const = 0;

        //! Performs a basic feed-forward operation to compute the output from a set of activation values.
        virtual const AZ::VectorN& Forward(const AZ::VectorN& activations) = 0;

        //! Accumulates the loss gradients given a loss function, an activation vector and a corresponding label vector.
        virtual void Reverse(LossFunctions lossFunction, const AZ::VectorN& activations, const AZ::VectorN& expected) = 0;

        //! Performs a gradient descent step and resets all gradient accumulators to zero.
        virtual void GradientDescent(float learningRate) = 0;
    };

    using INeuralNetworkPtr = AZStd::shared_ptr<INeuralNetwork>;
}
