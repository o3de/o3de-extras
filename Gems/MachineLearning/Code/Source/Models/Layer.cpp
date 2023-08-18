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

    const AZ::VectorN& Layer::Forward(const AZ::VectorN& activations)
    {
        m_lastInput = activations;
        m_output = m_biases;
        AZ::VectorMatrixMultiply(m_weights, m_lastInput, m_output);
        Activate(m_activationFunction, m_output, m_output);
        return m_output;
    }

    void Layer::AccumulateGradients(const AZ::VectorN& previousLayerGradients)
    {
        // Ensure our bias gradient vector is appropriately sized
        if (m_biasGradients.GetDimensionality() != m_outputSize)
        {
            m_biasGradients = AZ::VectorN::CreateZero(m_outputSize);
        }

        // Ensure our weight gradient matrix is appropriately sized
        if ((m_weightGradients.GetRowCount() != m_outputSize) || (m_weightGradients.GetColumnCount() != m_inputSize))
        {
            m_weightGradients = AZ::MatrixMxN::CreateZero(m_outputSize, m_inputSize);
        }

        // Ensure our backpropagation gradient vector is appropriately sized
        if (m_backpropagationGradients.GetDimensionality() != m_inputSize)
        {
            m_backpropagationGradients = AZ::VectorN::CreateZero(m_inputSize);
        }

        // Compute the partial derivatives of the output with respect to the activation function
        Activate_Derivative(m_activationFunction, m_output, previousLayerGradients, m_activationGradients);

        // Accumulate the partial derivatives of the weight matrix with respect to the loss function
        AZ::OuterProduct(m_activationGradients, m_lastInput, m_weightGradients);

        // Accumulate the partial derivatives of the bias vector with respect to the loss function
        m_biasGradients += m_activationGradients;

        // Accumulate the gradients to pass to the preceding layer for back-propagation
        AZ::VectorMatrixMultiplyLeft(m_activationGradients, m_weights, m_backpropagationGradients);
    }

    void Layer::ApplyGradients(float learningRate)
    {
        m_weights -= m_weightGradients * learningRate;
        m_biases -= m_biasGradients * learningRate;

        m_biasGradients.SetZero();
        m_weightGradients.SetZero();
        m_backpropagationGradients.SetZero();
    }

    void Layer::OnSizesChanged()
    {
        m_weights = AZ::MatrixMxN::CreateRandom(m_outputSize, m_inputSize);
        m_weights -= 0.5f; // It's preferable for efficient training to keep initial weights centered around zero

        m_biases = AZ::VectorN(m_outputSize, 0.01f);
        m_output = AZ::VectorN::CreateZero(m_outputSize);
    }
}
