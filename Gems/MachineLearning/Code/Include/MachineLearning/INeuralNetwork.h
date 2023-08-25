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
#include <MachineLearning/IInferenceContext.h>
#include <MachineLearning/ITrainingContext.h>
#include <AzCore/EBus/EBus.h>
#include <AzCore/std/string/string.h>

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

        //! Returns a name for the model.
        virtual AZStd::string GetName() const { return ""; }

        //! Returns the file where model parameters are stored.
        virtual AZStd::string GetAssetFile([[maybe_unused]] AssetTypes assetType) const { return ""; }

        //! Returns the number of input neurons the model supports.
        virtual AZStd::size_t GetInputDimensionality() const { return 0; }

        //! Returns the number of output neurons the model supports.
        virtual AZStd::size_t GetOutputDimensionality() const { return 0; }

        //! Returns the total number of layers in the network.
        virtual AZStd::size_t GetLayerCount() const { return 0; }

        //! Returns the total number of parameters in the neural network.
        virtual AZStd::size_t GetParameterCount() const { return 0; }

        // Returns a new inference context suitable for forward propagation operations.
        virtual IInferenceContextPtr CreateInferenceContext() { return nullptr; }

        // Returns a new training context suitable for back-propagation and gradient descent.
        virtual ITrainingContextPtr CreateTrainingContext() { return nullptr; }

        //! Performs a basic feed-forward operation to compute the output from a set of activation values.
        virtual const AZ::VectorN* Forward([[maybe_unused]] IInferenceContextPtr context, [[maybe_unused]] const AZ::VectorN& activations) { return nullptr; }

        //! Accumulates the loss gradients given a loss function, an activation vector and a corresponding label vector.
        virtual void Reverse([[maybe_unused]] ITrainingContextPtr context, [[maybe_unused]] LossFunctions lossFunction, [[maybe_unused]] const AZ::VectorN& activations, [[maybe_unused]] const AZ::VectorN& expected) {}

        //! Performs a gradient descent step and resets all gradient accumulators to zero.
        virtual void GradientDescent([[maybe_unused]] ITrainingContextPtr context, [[maybe_unused]] float learningRate) {}

        //! Loads the current model parameters from the associated asset file.
        virtual bool LoadModel() { return false; }

        //! Saves the current model parameters to the associated asset file.
        virtual bool SaveModel() { return false; }

        //! For intrusive_ptr support
        //! @{
        void add_ref() {}
        void release() {}
        //! @}
    };

    using INeuralNetworkPtr = AZStd::intrusive_ptr<INeuralNetwork>;
}
