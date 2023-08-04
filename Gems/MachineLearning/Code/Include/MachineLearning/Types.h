/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/Math/VectorN.h>

namespace MachineLearning
{
    enum class LossFunctions
    {
        MeanSquaredError
    };

    enum class ActivationFunctions
    {
        ReLU,
        Sigmoid,
        Linear
    };

    class Layer;

    struct LayerParams
    {
        AZ_TYPE_INFO(LayerParams, "{DD9A7E7C-8D11-4805-83CF-6A5262B4580C}");

        //! AzCore Reflection.
        //! @param context reflection context
        static void Reflect(class AZ::ReflectContext* context);

        LayerParams() = default;
        inline LayerParams(AZStd::size_t size, ActivationFunctions activationFunction)
            : m_layerSize(size)
            , m_activationFunction(activationFunction)
        {
        }

        AZStd::size_t m_layerSize = 0;
        ActivationFunctions m_activationFunction = ActivationFunctions::ReLU;
    };

    //using HiddenLayerParams = AZStd::vector<LayerParams>;
    using HiddenLayerParams = AZStd::vector<AZStd::size_t>;
}
