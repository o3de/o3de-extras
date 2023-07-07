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
                ;

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<Layer>("A single layer of a neural network", "")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &Layer::m_inputSize, "Input Size", "This value must match the output size of the previous layer, or the number of neurons in the activation layer if this is the first layer")
                    ->Attribute(AZ::Edit::Attributes::ChangeNotify, &Layer::OnSizesChanged)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &Layer::m_outputSize, "Output Size", "This value must match the input size of the next layer, if one exists")
                    ->Attribute(AZ::Edit::Attributes::ChangeNotify, &Layer::OnSizesChanged)
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
                Constructor<AZStd::size_t, AZStd::size_t>()->
                Attribute(AZ::Script::Attributes::Storage, AZ::Script::Attributes::StorageType::Value)
                ;
        }
    }

    Layer::Layer(AZStd::size_t activationDimensionality, AZStd::size_t layerDimensionality)
        : m_inputSize(activationDimensionality)
        , m_outputSize(layerDimensionality)
    {
        OnSizesChanged();
    }

    const AZ::VectorN& Layer::ActivateLayer(const AZ::VectorN& activations)
    {
        m_output = m_biases;
        AZ::VectorMatrixMultiply(m_weights, activations, m_output);
        return m_output;
    }

    void Layer::OnSizesChanged()
    {
        m_weights = AZ::MatrixMxN::CreateRandom(m_outputSize, m_inputSize);
        m_biases = AZ::VectorN::CreateRandom(m_outputSize);
        m_output = AZ::VectorN::CreateZero(m_outputSize);
    }
}
