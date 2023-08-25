/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/Math/MatrixMxN.h>
#include <AzNetworking/Serialization/ISerializer.h>
#include <MachineLearning/INeuralNetwork.h>

namespace MachineLearning
{
    // We separate out inference and training data to make multithreading models easier and more efficient.
    struct LayerInferenceData;
    struct LayerTrainingData;

    //! A class representing a single layer within a neural network.
    class Layer
    {
    public:

        AZ_TYPE_INFO(Layer, "{FB91E0A7-86C0-4431-83A8-04F8D8E1C9E2}");

        //! AzCore Reflection.
        //! @param context reflection context
        static void Reflect(AZ::ReflectContext* context);

        Layer() = default;
        Layer(Layer&&) = default;
        Layer(const Layer&) = default;
        Layer(ActivationFunctions activationFunction, AZStd::size_t activationDimensionality, AZStd::size_t layerDimensionality);
        ~Layer() = default;

        Layer& operator=(Layer&&) = default;
        Layer& operator=(const Layer&) = default;

        //! Performs a basic forward pass on this layer, outputs are stored in m_output.
        const AZ::VectorN& Forward(LayerInferenceData& inferenceData, const AZ::VectorN& activations);

        //! Performs a gradient computation against the provided expected output using the provided gradients from the previous layer.
        //! This method presumes that we've completed a forward pass immediately prior to fill all the relevant vectors
        void AccumulateGradients(LayerTrainingData& trainingData, LayerInferenceData& inferenceData, const AZ::VectorN& expected);

        //! Applies the current gradient values to the layers weights and biases and resets the gradient values for a new accumulation pass.
        void ApplyGradients(LayerTrainingData& trainingData, float learningRate);

        //! Base serialize method for all serializable structures or classes to implement.
        //! @param serializer ISerializer instance to use for serialization
        //! @return boolean true for success, false for serialization failure
        bool Serialize(AzNetworking::ISerializer& serializer);

        //! Returns the estimated size required to serialize this layer.
        AZStd::size_t EstimateSerializeSize() const;

        //! Updates layer internals for it's requested dimensionalities.
        void OnSizesChanged();

        // These are intentionally left public so that unit testing can exhaustively examine all layer state
        AZStd::size_t m_inputSize = 0;
        AZStd::size_t m_outputSize = 0;
        AZ::MatrixMxN m_weights;
        AZ::VectorN m_biases;
        ActivationFunctions m_activationFunction = ActivationFunctions::ReLU;
    };

    //! These values are written to during inference.
    struct LayerInferenceData
    {
        AZ::VectorN m_output;
    };

    //! These values are read and written during training.
    struct LayerTrainingData
    {
        // These values will only be populated if backward propagation is performed
        const AZ::VectorN* m_lastInput;
        AZ::VectorN m_activationGradients;
        AZ::VectorN m_biasGradients;
        AZ::MatrixMxN m_weightGradients;
        AZ::VectorN m_backpropagationGradients;
    };
}
