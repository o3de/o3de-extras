/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <Algorithms/Activations.h>
#include <AzCore/Math/SimdMath.h>
#include <AzCore/Math/MatrixMxN.h>

namespace MachineLearning
{
    AZStd::vector<AZ::Edit::EnumConstant<ActivationFunctions>> GetActivationEnumValues()
    {
        AZStd::vector<AZ::Edit::EnumConstant<ActivationFunctions>> values;
        values.emplace_back(ActivationFunctions::ReLU, "ReLU");
        values.emplace_back(ActivationFunctions::Sigmoid, "Sigmoid");
        values.emplace_back(ActivationFunctions::Softmax, "Softmax");
        values.emplace_back(ActivationFunctions::Linear, "Linear");
        return values;
    }

    void OneHotEncode(AZStd::size_t value, AZStd::size_t maxValue, AZ::VectorN& output)
    {
        AZ_Assert(value <= maxValue, "Requested one-hot encode of an out of range value");
        output.Resize(maxValue);
        output.SetZero();
        output.SetElement(value, 1.0f);
    }

    AZStd::size_t ArgMaxDecode(const AZ::VectorN& vector)
    {
        const AZStd::size_t numElements = vector.GetDimensionality();
        float maxValue = 0.0f;
        AZStd::size_t maxIndex = 0;
        for (AZStd::size_t iter = 0; iter < numElements; ++iter)
        {
            if (vector.GetElement(iter) > maxValue)
            {
                maxValue = vector.GetElement(iter);
                maxIndex = iter;
            }
        }
        return maxIndex;
    }

    void Activate(ActivationFunctions activationFunction, const AZ::VectorN& sourceVector, AZ::VectorN& output)
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
        case ActivationFunctions::Softmax:
            Softmax(sourceVector, output);
            break;
        case ActivationFunctions::Linear:
            Linear(sourceVector, output);
            break;
        }
    }

    void ReLU(const AZ::VectorN& sourceVector, AZ::VectorN& output)
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

    void Sigmoid(const AZ::VectorN& sourceVector, AZ::VectorN& output)
    {
        const AZ::Vector4 vecZero = AZ::Vector4::CreateZero();
        const AZ::Vector4 vecOne = AZ::Vector4::CreateOne();
        const AZ::Vector4 epsilon = AZ::Vector4(AZ::Constants::Tolerance);
        const AZStd::size_t numElements = sourceVector.GetVectorValues().size();
        output.Resize(sourceVector.GetDimensionality());
        for (AZStd::size_t iter = 0; iter < numElements; ++iter)
        {
            const AZ::Vector4& sourceElement = sourceVector.GetVectorValues()[iter];
            AZ::Vector4& outputElement = output.GetVectorValues()[iter];
            const AZ::Vector4 divisor = (vecOne + (-sourceElement).GetExpEstimate()).GetMax(epsilon);
            outputElement = vecOne / divisor;
            outputElement = outputElement.GetClamp(vecZero, vecOne);
        }
        output.FixLastVectorElement();
    }

    void Softmax(const AZ::VectorN& sourceVector, AZ::VectorN& output)
    {
        const AZ::Vector4 vecZero = AZ::Vector4::CreateZero();
        const AZ::Vector4 vecOne = AZ::Vector4::CreateOne();

        // Naive softmax is simply softmax(source) = exp(source) / sum(exp(source))
        // Here we apply the exp-normalization trick to avoid exp overflow
        // x = max(source)
        // y = exp(source - x)
        // softmax(source) = y / sum(y)
        const AZStd::size_t numElements = sourceVector.GetVectorValues().size();
        output.Resize(sourceVector.GetDimensionality());

        AZ::Vector4 max = sourceVector.GetVectorValues()[0];
        for (AZStd::size_t iter = 1; iter < numElements; ++iter)
        {
            max.GetMax(sourceVector.GetVectorValues()[iter]);
        }

        AZ::Vector4 partialSum = vecZero;
        for (AZStd::size_t iter = 0; iter < numElements; ++iter)
        {
            const AZ::Vector4& sourceElement = sourceVector.GetVectorValues()[iter];
            AZ::Vector4& outputElement = output.GetVectorValues()[iter];
            outputElement = (sourceElement - max).GetExpEstimate();
            outputElement = outputElement.GetClamp(vecZero, vecOne);
            partialSum += outputElement;
        }

        const float divisor = AZ::GetMax(1.0f / partialSum.Dot(vecOne), AZ::Constants::Tolerance);
        for (AZ::Vector4& element : output.GetVectorValues())
        {
            element = element * divisor;
        }
        output.FixLastVectorElement();
    }

    void Linear(const AZ::VectorN& sourceVector, AZ::VectorN& output)
    {
        if (&output != &sourceVector)
        {
            output = sourceVector;
        }
    }

    void Activate_Derivative(ActivationFunctions activationFunction, const AZ::VectorN& activationOutput, const AZ::VectorN& backGradients, AZ::VectorN& output)
    {
        output.Resize(activationOutput.GetDimensionality());
        switch (activationFunction)
        {
        case ActivationFunctions::ReLU:
            ReLU_Derivative(activationOutput, backGradients, output);
            break;
        case ActivationFunctions::Sigmoid:
            Sigmoid_Derivative(activationOutput, backGradients, output);
            break;
        case ActivationFunctions::Softmax:
            Softmax_Derivative(activationOutput, backGradients, output);
            break;
        case ActivationFunctions::Linear:
            Linear_Derivative(activationOutput, backGradients, output);
            break;
        }
    }

    void ReLU_Derivative(const AZ::VectorN& activationOutput, const AZ::VectorN& backGradients, AZ::VectorN& output)
    {
        const AZStd::size_t numElements = activationOutput.GetVectorValues().size();
        const AZ::Simd::Vec4::FloatType zero = AZ::Simd::Vec4::ZeroFloat();
        output.Resize(activationOutput.GetDimensionality());
        for (AZStd::size_t iter = 0; iter < numElements; ++iter)
        {
            const AZ::Vector4& activationElement = activationOutput.GetVectorValues()[iter];
            const AZ::Vector4& backGradientElement = backGradients.GetVectorValues()[iter];
            // 1's if > 0, 0's otherwise
            // Strictly greater than is required as any negative inputs in the original source vector will have been clamped to zero by activation
            const AZ::Simd::Vec4::FloatType mask = AZ::Simd::Vec4::CmpGt(activationElement.GetSimdValue(), zero);
            AZ::Vector4& outputElement = output.GetVectorValues()[iter];
            outputElement.SetSimdValue(AZ::Simd::Vec4::And(backGradientElement.GetSimdValue(), mask)); // Returns the backpropagated gradient if mask is non-zero, returns zero otherwise
        }
        output.FixLastVectorElement();
    }

    void Sigmoid_Derivative(const AZ::VectorN& activationOutput, const AZ::VectorN& backGradients, AZ::VectorN& output)
    {
        const AZStd::size_t numElements = activationOutput.GetVectorValues().size();
        const AZ::Vector4 vecOne = AZ::Vector4::CreateOne();
        output.Resize(activationOutput.GetDimensionality());
        for (AZStd::size_t iter = 0; iter < numElements; ++iter)
        {
            const AZ::Vector4& activationElement = activationOutput.GetVectorValues()[iter];
            const AZ::Vector4& backGradientElement = backGradients.GetVectorValues()[iter];
            AZ::Vector4& outputElement = output.GetVectorValues()[iter];
            outputElement = backGradientElement * activationElement * (vecOne - activationElement);
        }
        output.FixLastVectorElement();
    }

    void Softmax_Derivative(const AZ::VectorN& activationOutput, const AZ::VectorN& backGradients, AZ::VectorN& output)
    {
        // Note that this is completely unvectorized
        output.Resize(activationOutput.GetDimensionality());
        for (AZStd::size_t i = 0; i < activationOutput.GetDimensionality(); ++i)
        {
            float gradient = 0.0f;
            for (AZStd::size_t j = 0; j < activationOutput.GetDimensionality(); ++j)
            {
                const float ithElement = activationOutput.GetElement(i);
                const float jthElement = activationOutput.GetElement(j) * backGradients.GetElement(j);
                gradient += (i == j) ? (1.0f - ithElement) * jthElement : -ithElement * jthElement;
            }
            output.SetElement(i, gradient);
        }
    }

    void Linear_Derivative([[maybe_unused]] const AZ::VectorN& activationOutput, const AZ::VectorN& backGradients, AZ::VectorN& output)
    {
        output = backGradients;
    }
}
