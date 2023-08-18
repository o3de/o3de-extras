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
#include <AzCore/EBus/EBus.h>

namespace MachineLearning
{
    class Layer;

    class INeuralNetwork
    {
    public:

        AZ_RTTI(INeuralNetwork, "{64E5B5B1-4A7D-489D-9A29-D9510BB7E17A}");

        INeuralNetwork() = default;
        INeuralNetwork(INeuralNetwork&&) = default;
        INeuralNetwork(const INeuralNetwork&) = default;
        virtual ~INeuralNetwork() = default;

        INeuralNetwork& operator=(INeuralNetwork&&) = default;
        INeuralNetwork& operator=(const INeuralNetwork&) = default;

        //! Adds a new layer to the network.
        virtual void AddLayer([[maybe_unused]] AZStd::size_t layerDimensionality, [[maybe_unused]] ActivationFunctions activationFunction = ActivationFunctions::ReLU) {}

        //! Returns the total number of layers in the network.
        virtual AZStd::size_t GetLayerCount() const { return 0; }

        //! Retrieves a specific layer from the network indexed by the layerIndex.
        virtual Layer* GetLayer([[maybe_unused]] AZStd::size_t layerIndex) { return nullptr; }

        //! Returns the total number of parameters in the neural network.
        virtual AZStd::size_t GetParameterCount() const { return 0; }

        //! Performs a basic feed-forward operation to compute the output from a set of activation values.
        virtual const AZ::VectorN* Forward([[maybe_unused]] const AZ::VectorN& activations) { return nullptr; }

        //! Accumulates the loss gradients given a loss function, an activation vector and a corresponding label vector.
        virtual void Reverse([[maybe_unused]] LossFunctions lossFunction, [[maybe_unused]] const AZ::VectorN& activations, [[maybe_unused]] const AZ::VectorN& expected) {}

        //! Performs a gradient descent step and resets all gradient accumulators to zero.
        virtual void GradientDescent([[maybe_unused]] float learningRate) {}

        //! For intrusive_ptr support
        //! @{
        void add_ref() {}
        void release() {}
        //! @}
    };

    using INeuralNetworkPtr = AZStd::intrusive_ptr<INeuralNetwork>;
}
