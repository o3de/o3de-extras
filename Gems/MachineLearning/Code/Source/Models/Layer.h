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

namespace MachineLearning
{
    //! A class representing a single layer within a neural network.
    class Layer
    {
    public:

        AZ_TYPE_INFO(Layer, "{FB91E0A7-86C0-4431-83A8-04F8D8E1C9E2}");

        //! AzCore Reflection.
        //! @param context reflection context
        static void Reflect(AZ::ReflectContext* context);

        Layer() = default;
        Layer(AZStd::size_t activationDimensionality, AZStd::size_t layerDimensionality);

        const AZ::VectorN& ActivateLayer(const AZ::VectorN& activations);

        void OnSizesChanged();

        AZStd::size_t m_inputSize = 0;
        AZStd::size_t m_outputSize = 0;
        AZ::MatrixMxN m_weights;
        AZ::VectorN m_biases;
        AZ::VectorN m_output;
    };
}
