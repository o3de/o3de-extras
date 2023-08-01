/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <Nodes/CreateModel.h>
#include <Models/MultilayerPerceptron.h>

namespace MachineLearning
{
    INeuralNetworkPtr CreateModel::In(AZStd::size_t Inputneurons, AZStd::size_t Outputparams, HiddenLayerParams Hiddenlayers)
    {
        INeuralNetworkPtr result = AZStd::make_unique<MultilayerPerceptron>(Inputneurons);
        MultilayerPerceptron* modelPtr = static_cast<MultilayerPerceptron*>(result.get());
        for (auto layerParams : Hiddenlayers)
        {
            modelPtr->AddLayer(layerParams); //.m_layerSize, layerParams.m_activationFunction);
        }
        modelPtr->AddLayer(Outputparams); //.m_layerSize, Outputparams.m_activationFunction);
        return result;
    }
}
