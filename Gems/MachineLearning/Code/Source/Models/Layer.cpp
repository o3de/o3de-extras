/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <Models/MultilayerPerceptron.h>
#include <Algorithms/Activations.h>
#include <Algorithms/LossFunctions.h>
#include <AzCore/RTTI/RTTI.h>
#include <AzCore/RTTI/BehaviorContext.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/Console/IConsole.h>
#include <AzCore/Console/ILogger.h>
#include <random>

namespace MachineLearning
{
    AZ_CVAR(bool, ml_logGradients, false, nullptr, AZ::ConsoleFunctorFlags::Null, "Dumps some gradient metrics so they can be monitored during training");

    void AccumulateBiasGradients(AZ::VectorN& biasGradients, const AZ::VectorN& activationGradients, AZStd::size_t currentSamples)
    {
        AZ::Vector4 divisor(static_cast<float>(currentSamples));

        AZStd::vector<AZ::Vector4>& biasValues = biasGradients.GetVectorValues();
        const AZStd::vector<AZ::Vector4>& activationValues = activationGradients.GetVectorValues();
        for (AZStd::size_t iter = 0; iter < biasValues.size(); ++iter)
        {
            // average += (next - average) / samples
            biasValues[iter] += (activationValues[iter] - biasValues[iter]) / divisor;
        }
    }

    void AccumulateWeightGradients(const AZ::VectorN& activationGradients, const AZ::VectorN& lastInput, AZ::MatrixMxN& weightGradients, AZStd::size_t currentSamples)
    {
        // The following performs an outer product between activationGradients and lastInput
        // The reason we're not simply iteratively invoking OuterProduct is so that we can compute a more numerically stable average and preserve our gradients better over large batch sizes
        const AZ::Simd::Vec4::FloatType divisor = AZ::Simd::Vec4::Splat(static_cast<float>(currentSamples));
        for (AZStd::size_t colIter = 0; colIter < weightGradients.GetColumnGroups(); ++colIter)
        {
            AZ::Simd::Vec4::FloatType rhsElement = lastInput.GetVectorValues()[colIter].GetSimdValue();
            AZ::Simd::Vec4::FloatType splat0 = AZ::Simd::Vec4::SplatIndex0(rhsElement);
            AZ::Simd::Vec4::FloatType splat1 = AZ::Simd::Vec4::SplatIndex1(rhsElement);
            AZ::Simd::Vec4::FloatType splat2 = AZ::Simd::Vec4::SplatIndex2(rhsElement);
            AZ::Simd::Vec4::FloatType splat3 = AZ::Simd::Vec4::SplatIndex3(rhsElement);
            for (AZStd::size_t rowIter = 0; rowIter < weightGradients.GetRowGroups(); ++rowIter)
            {
                AZ::Simd::Vec4::FloatType lhsElement = activationGradients.GetVectorValues()[rowIter].GetSimdValue();
                AZ::Matrix4x4& outputElement = weightGradients.GetSubmatrix(rowIter, colIter);
                AZ::Simd::Vec4::FloatType next0 = AZ::Simd::Vec4::Sub(AZ::Simd::Vec4::Mul(lhsElement, splat0), outputElement.GetSimdValues()[0]);
                AZ::Simd::Vec4::FloatType next1 = AZ::Simd::Vec4::Sub(AZ::Simd::Vec4::Mul(lhsElement, splat1), outputElement.GetSimdValues()[1]);
                AZ::Simd::Vec4::FloatType next2 = AZ::Simd::Vec4::Sub(AZ::Simd::Vec4::Mul(lhsElement, splat2), outputElement.GetSimdValues()[2]);
                AZ::Simd::Vec4::FloatType next3 = AZ::Simd::Vec4::Sub(AZ::Simd::Vec4::Mul(lhsElement, splat3), outputElement.GetSimdValues()[3]);

                // average += (next - average) / samples
                outputElement.GetSimdValues()[0] = AZ::Simd::Vec4::Add(outputElement.GetSimdValues()[0], AZ::Simd::Vec4::Div(next0, divisor));
                outputElement.GetSimdValues()[1] = AZ::Simd::Vec4::Add(outputElement.GetSimdValues()[1], AZ::Simd::Vec4::Div(next1, divisor));
                outputElement.GetSimdValues()[2] = AZ::Simd::Vec4::Add(outputElement.GetSimdValues()[2], AZ::Simd::Vec4::Div(next2, divisor));
                outputElement.GetSimdValues()[3] = AZ::Simd::Vec4::Add(outputElement.GetSimdValues()[3], AZ::Simd::Vec4::Div(next3, divisor));
            }
        }
        weightGradients.FixUnusedElements();
    }

    void GetMinMaxElements(const AZ::VectorN& source, float& min, float& max)
    {
        const AZStd::vector<AZ::Vector4>& elements = source.GetVectorValues();
        if (!elements.empty())
        {
            AZ::Vector4 minimum = elements[0];
            AZ::Vector4 maximum = elements[0];
            for (AZStd::size_t i = 1; i < elements.size(); ++i)
            {
                minimum.GetMin(elements[i]);
                maximum.GetMax(elements[i]);
            }
            min = AZ::GetMin(AZ::GetMin(minimum.GetX(), minimum.GetY()), AZ::GetMin(minimum.GetZ(), minimum.GetW()));
            max = AZ::GetMax(AZ::GetMax(minimum.GetX(), minimum.GetY()), AZ::GetMax(minimum.GetZ(), minimum.GetW()));
        }
    }

    void GetMinMaxElements(AZ::MatrixMxN& source, float& min, float& max)
    {
        AZStd::vector<AZ::Matrix4x4>& elements = source.GetMatrixElements();
        if (!elements.empty())
        {
            AZ::Vector4 minimum = elements[0].GetRow(0);
            AZ::Vector4 maximum = elements[0].GetRow(0);
            for (AZStd::size_t i = 1; i < elements.size(); ++i)
            {
                for (int32_t j = 0; j < 4; ++j)
                {
                    minimum = minimum.GetMin(elements[i].GetRow(j));
                    maximum = maximum.GetMax(elements[i].GetRow(j));
                }
            }
            min = AZ::GetMin(AZ::GetMin(minimum.GetX(), minimum.GetY()), AZ::GetMin(minimum.GetZ(), minimum.GetW()));
            max = AZ::GetMax(AZ::GetMax(minimum.GetX(), minimum.GetY()), AZ::GetMax(minimum.GetZ(), minimum.GetW()));
        }
    }

    void Layer::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<Layer>()
                ->Version(1)
                ->Field("InputSize", &Layer::m_inputSize)
                ->Field("OutputSize", &Layer::m_outputSize)
                ->Field("Weights", &Layer::m_weights)
                ->Field("Biases", &Layer::m_biases)
                ->Field("ActivationFunction", &Layer::m_activationFunction)
                ;

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<Layer>("A single layer of a neural network", "")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &Layer::m_outputSize, "Layer Size", "The number of neurons the layer should have")
                    ->Attribute(AZ::Edit::Attributes::ChangeNotify, &Layer::OnSizesChanged)
                    ->DataElement(AZ::Edit::UIHandlers::ComboBox, &Layer::m_activationFunction, "Activation Function", "The activation function applied to this layer")
                        ->Attribute(AZ::Edit::Attributes::EnumValues, &GetActivationEnumValues)
                    ;
            }
        }

        auto behaviorContext = azrtti_cast<AZ::BehaviorContext*>(context);
        if (behaviorContext)
        {
            behaviorContext->Class<Layer>()->
                Attribute(AZ::Script::Attributes::Scope, AZ::Script::Attributes::ScopeFlags::Common)->
                Attribute(AZ::Script::Attributes::Module, "machineLearning")->
                Attribute(AZ::Script::Attributes::ExcludeFrom, AZ::Script::Attributes::ExcludeFlags::ListOnly)->
                Constructor<ActivationFunctions, AZStd::size_t, AZStd::size_t>()->
                Attribute(AZ::Script::Attributes::Storage, AZ::Script::Attributes::StorageType::Value)->
                Property("InputSize", BehaviorValueProperty(&Layer::m_inputSize))->
                Property("OutputSize", BehaviorValueProperty(&Layer::m_outputSize))->
                Property("ActivationFunction", BehaviorValueProperty(&Layer::m_activationFunction))
                ;
        }
    }

    Layer::Layer(ActivationFunctions activationFunction, AZStd::size_t activationDimensionality, AZStd::size_t layerDimensionality)
        : m_activationFunction(activationFunction)
        , m_inputSize(activationDimensionality)
        , m_outputSize(layerDimensionality)
    {
        OnSizesChanged();
    }

    const AZ::VectorN& Layer::Forward(LayerInferenceData& inferenceData, const AZ::VectorN& activations)
    {
        inferenceData.m_output = m_biases;
        AZ::VectorMatrixMultiply(m_weights, activations, inferenceData.m_output);
        Activate(m_activationFunction, inferenceData.m_output, inferenceData.m_output);
        return inferenceData.m_output;
    }

    void Layer::AccumulateGradients(AZStd::size_t samples, LayerTrainingData& trainingData, LayerInferenceData& inferenceData, const AZ::VectorN& previousLayerGradients)
    {
        // Ensure our bias gradient vector is appropriately sized
        if (trainingData.m_biasGradients.GetDimensionality() != m_outputSize)
        {
            trainingData.m_biasGradients = AZ::VectorN::CreateZero(m_outputSize);
        }

        // Ensure our weight gradient matrix is appropriately sized
        if ((trainingData.m_weightGradients.GetRowCount() != m_outputSize) || (trainingData.m_weightGradients.GetColumnCount() != m_inputSize))
        {
            trainingData.m_weightGradients = AZ::MatrixMxN::CreateZero(m_outputSize, m_inputSize);
        }

        // Ensure our backpropagation gradient vector is appropriately sized
        if (trainingData.m_backpropagationGradients.GetDimensionality() != m_inputSize)
        {
            trainingData.m_backpropagationGradients = AZ::VectorN::CreateZero(m_inputSize);
        }

        // Compute the partial derivatives of the output with respect to the activation function
        Activate_Derivative(m_activationFunction, inferenceData.m_output, previousLayerGradients, trainingData.m_activationGradients);

        // Accumulate the partial derivatives of the weight matrix with respect to the loss function
        AccumulateWeightGradients(trainingData.m_activationGradients, *trainingData.m_lastInput, trainingData.m_weightGradients, samples);

        // Accumulate the partial derivatives of the bias vector with respect to the loss function
        AccumulateBiasGradients(trainingData.m_biasGradients, trainingData.m_activationGradients, samples);

        // Accumulate the gradients to pass to the preceding layer for back-propagation
        AZ::VectorMatrixMultiplyLeft(trainingData.m_activationGradients, m_weights, trainingData.m_backpropagationGradients);

        if (ml_logGradients)
        {
            float min = 0.f;
            float max = 0.f;
            GetMinMaxElements(trainingData.m_weightGradients, min, max);
            AZLOG_INFO("Weight gradients: min value %f, max value %f", min, max);

            GetMinMaxElements(trainingData.m_biasGradients, min, max);
            AZLOG_INFO("Bias gradients: min value %f, max value %f", min, max);

            GetMinMaxElements(trainingData.m_backpropagationGradients, min, max);
            AZLOG_INFO("Back-propagation gradients: min value %f, max value %f", min, max);

            //for (AZStd::size_t i = 0; i < trainingData.m_weightGradients.GetRowCount(); ++i)
            //{
            //    for (AZStd::size_t j = 0; j < trainingData.m_weightGradients.GetColumnCount(); ++j)
            //    {
            //        AZLOG_INFO("Weight %ux%u : %f", i, j, trainingData.m_weightGradients.GetElement(i, j));
            //    }
            //}
            //for (AZStd::size_t i = 0; i < trainingData.m_biasGradients.GetDimensionality(); ++i)
            //{
            //    AZLOG_INFO("Bias %u : %f", i, trainingData.m_biasGradients.GetElement(i));
            //}
        }
    }

    void Layer::ApplyGradients(LayerTrainingData& trainingData, float learningRate)
    {
        m_weights -= trainingData.m_weightGradients * learningRate;
        m_biases -= trainingData.m_biasGradients * learningRate;

        trainingData.m_biasGradients.SetZero();
        trainingData.m_weightGradients.SetZero();
        trainingData.m_backpropagationGradients.SetZero();
    }

    bool Layer::Serialize(AzNetworking::ISerializer& serializer)
    {
        return serializer.Serialize(m_inputSize, "inputSize")
            && serializer.Serialize(m_outputSize, "outputSize")
            && serializer.Serialize(m_weights, "weights")
            && serializer.Serialize(m_biases, "biases")
            && serializer.Serialize(m_activationFunction, "activationFunction");
    }

    AZStd::size_t Layer::EstimateSerializeSize() const
    {
        const AZStd::size_t padding = 64; // 64 bytes of extra padding just in case
        return padding
             + sizeof(m_inputSize)
             + sizeof(m_outputSize)
             + sizeof(AZStd::size_t) // for m_weights row count
             + sizeof(AZStd::size_t) // for m_weights column count
             + sizeof(AZStd::size_t) // for m_weights vector size
             + sizeof(float) * m_outputSize * m_inputSize // m_weights buffer
             + sizeof(AZStd::size_t) // for m_biases dimensionality
             + sizeof(AZStd::size_t) // for m_biases vector size
             + sizeof(float) * m_outputSize // m_biases buffer
             + sizeof(m_activationFunction);
    }

    void Layer::OnSizesChanged()
    {
        // Specifically for ReLU, we use Kaiming He initialization as this is optimal for convergence
        // For other activation functions we just use a standard normal distribution
        float standardDeviation = (m_activationFunction == ActivationFunctions::ReLU) ? 2.0f / m_inputSize 
                                                                                      : 1.0f / m_inputSize;
        std::random_device rd{};
        std::mt19937 gen{ rd() };
        auto dist = std::normal_distribution<float>{ 0.0f, standardDeviation };
        m_weights.Resize(m_outputSize, m_inputSize);
        for (AZStd::size_t row = 0; row < m_weights.GetRowCount(); ++row)
        {
            for (AZStd::size_t col = 0; col < m_weights.GetRowCount(); ++col)
            {
                m_weights.SetElement(row, col, dist(gen));
            }
        }

        m_biases = AZ::VectorN(m_outputSize, 0.01f);
    }
}
