/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <Algorithms/LossFunctions.h>
#include <AzCore/Math/SimdMath.h>

namespace MachineLearning
{
    inline float ComputeTotalCost(LossFunctions lossFunction, const AZ::VectorN& expected, const AZ::VectorN& actual)
    {
        AZ::VectorN costs;
        ComputeLoss(lossFunction, expected, actual, costs);
        AZ::Vector4 accumulator = AZ::Vector4::CreateZero();
        for (const AZ::Vector4& element : costs.GetVectorValues())
        {
            accumulator += element;
        }
        return accumulator.Dot(AZ::Vector4::CreateOne());
    }

    inline void ComputeLoss(LossFunctions costFunction, const AZ::VectorN& expected, const AZ::VectorN& actual, AZ::VectorN& output)
    {
        AZ_Assert(expected.GetDimensionality() == actual.GetDimensionality(), "The dimensionality of expected and actual must match");
        output.Resize(actual.GetDimensionality());
        switch (costFunction)
        {
        case LossFunctions::MeanSquaredError:
            MeanSquaredError(expected, actual, output);
            break;
        case LossFunctions::CrossEntropyLoss:
            CrossEntropyLoss(expected, actual, output);
            break;
        }
    }

    inline void MeanSquaredError(const AZ::VectorN& expected, const AZ::VectorN& actual, AZ::VectorN& output)
    {
        output = 0.5f * (actual - expected).GetSquare();
    }

    inline void CrossEntropyLoss(const AZ::VectorN& expected, const AZ::VectorN& actual, AZ::VectorN& output)
    {
        output = -(expected / actual) + (1.0f - expected) / (1.0f - actual);
    }

    inline void ComputeLoss_Derivative(LossFunctions costFunction, const AZ::VectorN& expected, const AZ::VectorN& actual, AZ::VectorN& output)
    {
        AZ_Assert(expected.GetDimensionality() == actual.GetDimensionality(), "The dimensionality of expected and actual must match");
        output.Resize(actual.GetDimensionality());
        switch (costFunction)
        {
        case LossFunctions::MeanSquaredError:
            MeanSquaredError_Derivative(expected, actual, output);
            break;
        case LossFunctions::CrossEntropyLoss:
            CrossEntropyLoss_Derivative(expected, actual, output);
            break;
        }
    }

    inline void MeanSquaredError_Derivative(const AZ::VectorN& expected, const AZ::VectorN& actual, AZ::VectorN& output)
    {
        output = (expected - actual);
    }

    inline void CrossEntropyLoss_Derivative(const AZ::VectorN& expected, const AZ::VectorN& actual, AZ::VectorN& output)
    {
        output = -(expected / actual) + (1.0f - expected) / (1.0f - actual);
    }
}
