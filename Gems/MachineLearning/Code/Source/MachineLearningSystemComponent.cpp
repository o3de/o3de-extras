/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "MachineLearningSystemComponent.h"
#include <MachineLearning/MachineLearningTypeIds.h>
#include <MachineLearning/Types.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/RTTI/BehaviorContext.h>
#include <AzCore/Preprocessor/EnumReflectUtils.h>
#include <Algorithms/Activations.h>
#include <Assets/MnistDataLoader.h>
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
    AZ_ENUM_DEFINE_REFLECT_UTILITIES(ActivationFunctions);
    AZ_ENUM_DEFINE_REFLECT_UTILITIES(LossFunctions);

    AZ_COMPONENT_IMPL(MachineLearningSystemComponent, "MachineLearningSystemComponent", MachineLearningSystemComponentTypeId);

    void MachineLearningSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<MachineLearningSystemComponent, AZ::Component>()->Version(0);
            serializeContext->Class<ILabeledTrainingData>()->Version(0);
            serializeContext->Class<INeuralNetwork>()->Version(0);
            serializeContext->RegisterGenericType<INeuralNetworkPtr>();
            serializeContext->RegisterGenericType<ILabeledTrainingDataPtr>();
        }

        if (auto behaviorContext = azrtti_cast<AZ::BehaviorContext*>(context))
        {
            behaviorContext->Class<MachineLearningSystemComponent>();
            behaviorContext->Class<INeuralNetwork>("INeuralNetwork")->
                Attribute(AZ::Script::Attributes::Scope, AZ::Script::Attributes::ScopeFlags::Common)->
                Attribute(AZ::Script::Attributes::Module, "machineLearning")->
                Attribute(AZ::Script::Attributes::ExcludeFrom, AZ::Script::Attributes::ExcludeFlags::ListOnly)->
                Constructor<>()->
                Attribute(AZ::Script::Attributes::Storage, AZ::Script::Attributes::StorageType::Value)->
                Method("GetName", &INeuralNetwork::GetName)
                ;
        }

        Layer::Reflect(context);
        MnistDataLoader::Reflect(context);
        MultilayerPerceptron::Reflect(context);
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

    void MachineLearningSystemComponent::RegisterModel(INeuralNetworkPtr model)
    {
        m_registeredModels.emplace(model);
    }

    void MachineLearningSystemComponent::UnregisterModel(INeuralNetworkPtr model)
    {
        m_registeredModels.erase(model);
    }

    ModelSet& MachineLearningSystemComponent::GetModelSet()
    {
        return m_registeredModels;
    }
}
