/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/Math/MatrixMxN.h>
#include <MachineLearning/INeuralNetwork.h>
#include <Models/Layer.h>

namespace MachineLearning
{
    //! This is a basic multilayer perceptron neural network capable of basic training and feed forward operations.
    class MultilayerPerceptron
        : public INeuralNetwork
    {
    public:

        AZ_RTTI(MultilayerPerceptron, "{E12EF761-41A5-48C3-BF55-7179B280D45F}", INeuralNetwork);

        //! AzCore Reflection.
        //! @param context reflection context
        static void Reflect(AZ::ReflectContext* context);

        MultilayerPerceptron() = default;
        MultilayerPerceptron(MultilayerPerceptron&&) = default;
        MultilayerPerceptron(const MultilayerPerceptron&) = default;
        MultilayerPerceptron(AZStd::size_t activationCount);
        virtual ~MultilayerPerceptron() = default;

        MultilayerPerceptron& operator=(MultilayerPerceptron&&) = default;
        MultilayerPerceptron& operator=(const MultilayerPerceptron&) = default;

        //! INeuralNetwork interface
        //! @{
        void AddLayer(AZStd::size_t layerDimensionality, ActivationFunctions activationFunction = ActivationFunctions::ReLU) override;
        AZStd::size_t GetLayerCount() const override;
        Layer* GetLayer(AZStd::size_t layerIndex) override;
        AZStd::size_t GetParameterCount() const override;
        const AZ::VectorN* Forward(const AZ::VectorN& activations) override;
        void Reverse(LossFunctions lossFunction, const AZ::VectorN& activations, const AZ::VectorN& expected) override;
        void GradientDescent(float learningRate) override;
        //! @}

    private:

        void OnActivationCountChanged();

        //! The number of neurons in the activation layer.
        AZStd::size_t m_activationCount = 0;

        //! The number of accumulated training samples.
        AZStd::size_t m_trainingSampleSize = 0;

        //! The set of layers in the network.
        AZStd::vector<Layer> m_layers;
    };
}
