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
#include <random>

namespace MachineLearning
{
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

    void Layer::AccumulateGradients(LayerTrainingData& trainingData, LayerInferenceData& inferenceData, const AZ::VectorN& previousLayerGradients)
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
        AZ::OuterProduct(trainingData.m_activationGradients, *trainingData.m_lastInput, trainingData.m_weightGradients);

        // Accumulate the partial derivatives of the bias vector with respect to the loss function
        trainingData.m_biasGradients += trainingData.m_activationGradients;

        // Accumulate the gradients to pass to the preceding layer for back-propagation
        AZ::VectorMatrixMultiplyLeft(trainingData.m_activationGradients, m_weights, trainingData.m_backpropagationGradients);
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
        // Specifically for ReLU, we use Kaiming He initialization as this is proven optimal for convergence
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
