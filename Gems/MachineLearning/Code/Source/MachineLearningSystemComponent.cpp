/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "MachineLearningSystemComponent.h"
#include <MachineLearning/MachineLearningTypeIds.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/RTTI/BehaviorContext.h>
#include <Models/Layer.h>
#include <Models/MultilayerPerceptron.h>
#include <AutoGenNodeableRegistry.generated.h>

static ScriptCanvas::MachineLearningPrivateObjectNodeableRegistry s_MachineLearningPrivateObjectNodeableRegistry;

namespace AZ
{
    AZ_TYPE_INFO_SPECIALIZE(MachineLearning::ActivationFunctions, "{2ABF758E-CA69-41AC-BC95-B47AD7DEA31B}");
    AZ_TYPE_INFO_SPECIALIZE(MachineLearning::LossFunctions, "{18098C74-9AD0-4F1D-8093-545344620AD1}");
}

namespace MachineLearning
{
    AZ_COMPONENT_IMPL(MachineLearningSystemComponent, "MachineLearningSystemComponent", MachineLearningSystemComponentTypeId);

    void LayerParams::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<LayerParams>()
                ->Version(1)
                ->Field("Size", &LayerParams::m_layerSize)
                ->Field("ActivationFunction", &LayerParams::m_activationFunction)
                ;

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<LayerParams>("Parameters defining a single layer of a neural network", "")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &LayerParams::m_layerSize, "Layer Size", "The number of neurons this layer should have")
                    ->DataElement(AZ::Edit::UIHandlers::ComboBox, &LayerParams::m_activationFunction, "Activation Function", "The activation function applied to this layer")
                    ;
            }
        }

        auto behaviorContext = azrtti_cast<AZ::BehaviorContext*>(context);
        if (behaviorContext)
        {
            behaviorContext->Class<LayerParams>()->
                Attribute(AZ::Script::Attributes::Scope, AZ::Script::Attributes::ScopeFlags::Common)->
                Attribute(AZ::Script::Attributes::Module, "machineLearning")->
                Attribute(AZ::Script::Attributes::ExcludeFrom, AZ::Script::Attributes::ExcludeFlags::ListOnly)->
                Constructor<AZStd::size_t, ActivationFunctions>()->
                Attribute(AZ::Script::Attributes::Storage, AZ::Script::Attributes::StorageType::Value)
                ;
        }
    }

    void MachineLearningSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<MachineLearningSystemComponent, AZ::Component>()->Version(0);
            serializeContext->Class<Layer>()->Version(0);
            serializeContext->Class<MultilayerPerceptron>()->Version(0);
            serializeContext->Class<INeuralNetwork>()->Version(0);
            serializeContext->Class<LayerParams>();
            serializeContext->RegisterGenericType<INeuralNetworkPtr>();
            serializeContext->RegisterGenericType<HiddenLayerParams>();
        }

        if (auto behaviorContext = azrtti_cast<AZ::BehaviorContext*>(context))
        {
            behaviorContext->Class<MachineLearningSystemComponent>();
            behaviorContext->Class<LayerParams>();
            behaviorContext->Class<Layer>();
            behaviorContext->Class<MultilayerPerceptron>();

            behaviorContext
                ->Enum<static_cast<int>(ActivationFunctions::Linear)>("Linear activation function")
                ->Enum<static_cast<int>(ActivationFunctions::ReLU)>("ReLU activation function");

            behaviorContext
                ->Enum<static_cast<int>(LossFunctions::MeanSquaredError)>("Quadratic cost function")
                ->Enum<static_cast<int>(LossFunctions::CrossEntropyLoss)>("Cross entropy loss function");
        }
    }

    void MachineLearningSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("MachineLearningService"));
    }

    void MachineLearningSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("MachineLearningService"));
    }

    void MachineLearningSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
    }

    void MachineLearningSystemComponent::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
    }

    MachineLearningSystemComponent::MachineLearningSystemComponent()
    {
        if (MachineLearningInterface::Get() == nullptr)
        {
            MachineLearningInterface::Register(this);
        }
    }

    MachineLearningSystemComponent::~MachineLearningSystemComponent()
    {
        if (MachineLearningInterface::Get() == this)
        {
            MachineLearningInterface::Unregister(this);
        }
    }

    void MachineLearningSystemComponent::Init()
    {
    }

    void MachineLearningSystemComponent::Activate()
    {
        MachineLearningRequestBus::Handler::BusConnect();
    }

    void MachineLearningSystemComponent::Deactivate()
    {
        MachineLearningRequestBus::Handler::BusDisconnect();
    }
} // namespace MachineLearning
