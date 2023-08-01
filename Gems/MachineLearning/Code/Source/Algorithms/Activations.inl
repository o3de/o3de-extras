/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <Algorithms/Activations.h>
#include <AzCore/Math/SimdMath.h>

namespace MachineLearning
{
    inline void Activate(ActivationFunctions activationFunction, const AZ::VectorN& sourceVector, AZ::VectorN& output)
    {
        output.Resize(sourceVector.GetDimensionality());
        switch (activationFunction)
        {
        case ActivationFunctions::ReLU:
            ReLU(sourceVector, output);
            break;
        case ActivationFunctions::Sigmoid:
            Sigmoid(sourceVector, output);
            break;
        case ActivationFunctions::Linear:
            Linear(sourceVector, output);
            break;
        }
    }

    inline void ReLU(const AZ::VectorN& sourceVector, AZ::VectorN& output)
    {
        const AZStd::size_t numElements = sourceVector.GetVectorValues().size();
        const AZ::Simd::Vec4::FloatType zero = AZ::Simd::Vec4::ZeroFloat();
        output.Resize(sourceVector.GetDimensionality());
        for (AZStd::size_t iter = 0; iter < numElements; ++iter)
        {
            const AZ::Vector4& sourceElement = sourceVector.GetVectorValues()[iter];
            const AZ::Simd::Vec4::FloatType mask = AZ::Simd::Vec4::CmpGtEq(sourceElement.GetSimdValue(), zero); // 1's if >= 0, 0's otherwise
            AZ::Vector4& outputElement = output.GetVectorValues()[iter];
            outputElement.SetSimdValue(AZ::Simd::Vec4::And(sourceElement.GetSimdValue(), mask)); // Zeros out negative elements
        }
        output.FixLastVectorElement();
    }

    inline void Sigmoid(const AZ::VectorN& sourceVector, AZ::VectorN& output)
    {
        const AZStd::size_t numElements = sourceVector.GetVectorValues().size();
        const AZ::Vector4 vecOne = AZ::Vector4::CreateOne();
        output.Resize(sourceVector.GetDimensionality());
        for (AZStd::size_t iter = 0; iter < numElements; ++iter)
        {
            const AZ::Vector4& sourceElement = sourceVector.GetVectorValues()[iter];
            AZ::Vector4& outputElement = output.GetVectorValues()[iter];
            outputElement = vecOne / (vecOne + (-sourceElement).GetExpEstimate());
        }
        output.FixLastVectorElement();
    }

    inline void Linear(const AZ::VectorN& sourceVector, AZ::VectorN& output)
    {
        if (&output != &sourceVector)
        {
            output = sourceVector;
        }
    }

    inline void Activate_Derivative(ActivationFunctions activationFunction, const AZ::VectorN& activationOutput, AZ::VectorN& output)
    {
        output.Resize(activationOutput.GetDimensionality());
        switch (activationFunction)
        {
        case ActivationFunctions::ReLU:
            ReLU_Derivative(activationOutput, output);
            break;
        case ActivationFunctions::Sigmoid:
            Sigmoid_Derivative(activationOutput, output);
            break;
        case ActivationFunctions::Linear:
            Linear_Derivative(activationOutput, output);
            break;
        }
    }

    inline void ReLU_Derivative(const AZ::VectorN& activationOutput, AZ::VectorN& output)
    {
        const AZStd::size_t numElements = activationOutput.GetVectorValues().size();
        const AZ::Simd::Vec4::FloatType zero = AZ::Simd::Vec4::ZeroFloat();
        const AZ::Simd::Vec4::FloatType one = AZ::Simd::Vec4::Splat(1.0f);
        output.Resize(activationOutput.GetDimensionality());
        for (AZStd::size_t iter = 0; iter < numElements; ++iter)
        {
            const AZ::Vector4& sourceElement = activationOutput.GetVectorValues()[iter];
            // 1's if > 0, 0's otherwise
            // Strictly greater than is required as any negative inputs in the original source vector will have been clamped to zero by activation
            const AZ::Simd::Vec4::FloatType mask = AZ::Simd::Vec4::CmpGt(sourceElement.GetSimdValue(), zero);
            AZ::Vector4& outputElement = output.GetVectorValues()[iter];
            outputElement.SetSimdValue(AZ::Simd::Vec4::And(one, mask)); // Returns one if mask is non-zero, returns zero otherwise
        }
        output.FixLastVectorElement();
    }

    inline void Sigmoid_Derivative(const AZ::VectorN& activationOutput, AZ::VectorN& output)
    {
        const AZStd::size_t numElements = activationOutput.GetVectorValues().size();
        const AZ::Vector4 vecOne = AZ::Vector4::CreateOne();
        output.Resize(activationOutput.GetDimensionality());
        for (AZStd::size_t iter = 0; iter < numElements; ++iter)
        {
            const AZ::Vector4& activationElement = activationOutput.GetVectorValues()[iter];
            AZ::Vector4& outputElement = output.GetVectorValues()[iter];
            outputElement = activationElement * (vecOne - activationElement);
        }
        output.FixLastVectorElement();
    }

    inline void Linear_Derivative(const AZ::VectorN& activationOutput, AZ::VectorN& output)
    {
        output = AZ::VectorN(activationOutput.GetDimensionality(), 1.0f);
    }
}
