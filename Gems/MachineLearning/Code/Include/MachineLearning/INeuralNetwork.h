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
    enum class CostFunctions
    {
        Quadratic
    };

    class INeuralNetwork
    {
    public:

        AZ_TYPE_INFO(INeuralNetwork, "{64E5B5B1-4A7D-489D-9A29-D9510BB7E17A}");

        virtual ~INeuralNetwork() = default;

        //! Returns the total number of parameters in the neural network.
        virtual AZStd::size_t GetParameterCount() const = 0;

        //! Performs a basic feed-forward operation to compute the output from a set of activation values.
        virtual const AZ::VectorN& FeedForward(const AZ::VectorN& activations) = 0;

        //! Given a set of activations and an expected output, computes the cost of the 
        virtual float ComputeCost(const AZ::VectorN& activations, const AZ::VectorN& expectedOutput, CostFunctions costFunction) = 0;
    };

    using INeuralNetworkPtr = AZStd::shared_ptr<INeuralNetwork>;
    using HiddenLayerParams = AZStd::vector<AZStd::size_t>;
}
