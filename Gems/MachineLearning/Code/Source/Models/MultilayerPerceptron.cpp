/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <Models/MultilayerPerceptron.h>
#include <AzCore/RTTI/RTTI.h>
#include <AzCore/RTTI/BehaviorContext.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>

namespace MachineLearning
{
    void MultilayerPerceptron::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<MultilayerPerceptron>()
                ->Version(1)
                ->Field("ActivationCount", &MultilayerPerceptron::m_activationCount)
                ->Field("Layers", &MultilayerPerceptron::m_layers)
                ;

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<MultilayerPerceptron>("A basic multilayer perceptron class", "")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &MultilayerPerceptron::m_activationCount, "Activation Count", "The number of neurons in the activation layer")
                    ->Attribute(AZ::Edit::Attributes::ChangeNotify, &MultilayerPerceptron::OnActivationCountChanged)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &MultilayerPerceptron::m_layers, "Layers", "The layers of the neural network")
                    ->Attribute(AZ::Edit::Attributes::ChangeNotify, &MultilayerPerceptron::OnActivationCountChanged)
                    ;
            }
        }

        auto behaviorContext = azrtti_cast<AZ::BehaviorContext*>(context);
        if (behaviorContext)
        {
            behaviorContext->Class<MultilayerPerceptron>()->
                Attribute(AZ::Script::Attributes::Scope, AZ::Script::Attributes::ScopeFlags::Common)->
                Attribute(AZ::Script::Attributes::Module, "machineLearning")->
                Attribute(AZ::Script::Attributes::ExcludeFrom, AZ::Script::Attributes::ExcludeFlags::ListOnly)->
                Constructor<AZStd::size_t>()->
                Attribute(AZ::Script::Attributes::Storage, AZ::Script::Attributes::StorageType::Value)->
                Method("AddLayer", &MultilayerPerceptron::AddLayer)->
                Method("GetLayerCount", &MultilayerPerceptron::GetLayerCount)->
                Method("GetLayer", &MultilayerPerceptron::GetLayer)->
                Method("Forward", &MultilayerPerceptron::Forward)->
                Method("Reverse", &MultilayerPerceptron::Reverse)
                ;
        }
    }

    MultilayerPerceptron::MultilayerPerceptron(AZStd::size_t activationCount)
        : m_activationCount(activationCount)
    {
    }

    void MultilayerPerceptron::AddLayer(AZStd::size_t layerDimensionality, ActivationFunctions activationFunction)
    {
        AZStd::size_t lastLayerDimensionality = m_activationCount;
        if (!m_layers.empty())
        {
            lastLayerDimensionality = m_layers.back().m_biases.GetDimensionality();
        }
        m_layers.push_back(AZStd::move(Layer(activationFunction, lastLayerDimensionality, layerDimensionality)));
    }

    AZStd::size_t MultilayerPerceptron::GetLayerCount() const
    {
        return m_layers.size();
    }

    Layer& MultilayerPerceptron::GetLayer(AZStd::size_t layerIndex)
    {
        return m_layers[layerIndex];
    }

    AZStd::size_t MultilayerPerceptron::GetParameterCount() const
    {
        AZStd::size_t parameterCount = 0;
        for (const Layer& layer : m_layers)
        {
            parameterCount += layer.m_inputSize * layer.m_outputSize + layer.m_outputSize;
        }
        return parameterCount;
    }

    const AZ::VectorN& MultilayerPerceptron::Forward(const AZ::VectorN& activations)
    {
        const AZ::VectorN* lastLayerOutput = &activations;
        for (Layer& layer : m_layers)
        {
            layer.Forward(*lastLayerOutput);
            lastLayerOutput = &layer.m_output;
        }
        return *lastLayerOutput;
    }

    void MultilayerPerceptron::Reverse(LossFunctions lossFunction, const AZ::VectorN& activations, const AZ::VectorN& expected)
    {
        // First feed-forward the activations to get our current model predictions
        const AZ::VectorN& output = Forward(activations);

        // Compute the partial derivatives of the loss function with respect to the final layer output
        AZ::VectorN costGradients;
        ComputeLoss_Derivative(lossFunction, output, expected, costGradients);

        for (auto iter = m_layers.rbegin(); iter != m_layers.rend(); ++iter)
        {
            iter->AccumulateGradients(costGradients);
            costGradients = iter->m_backpropagationGradients;
        }
    }

    void MultilayerPerceptron::GradientDescent(float learningRate)
    {
        for (auto iter = m_layers.rbegin(); iter != m_layers.rend(); ++iter)
        {
            iter->ApplyGradients(learningRate);
        }
    }

    void MultilayerPerceptron::OnActivationCountChanged()
    {
        AZStd::size_t lastLayerDimensionality = m_activationCount;
        for (Layer& layer : m_layers)
        {
            layer.m_inputSize = lastLayerDimensionality;
            layer.OnSizesChanged();
            lastLayerDimensionality = layer.m_outputSize;
        }
    }
}
