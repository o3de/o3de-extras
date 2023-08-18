/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <Components/MultilayerPerceptronComponent.h>
#include <AzCore/RTTI/RTTI.h>
#include <AzCore/RTTI/BehaviorContext.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>

namespace MachineLearning
{
    void MultilayerPerceptronComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<MultilayerPerceptronComponent>()
                ->Version(0)
                ->Field("Model", &MultilayerPerceptronComponent::m_model)
                ;

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<MultilayerPerceptronComponent>("Multilayer Perceptron", "")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "MachineLearning")
                    ->Attribute(AZ::Edit::Attributes::Icon, "Editor/Icons/Components/NeuralNetwork.svg")
                    ->Attribute(AZ::Edit::Attributes::ViewportIcon, "Editor/Icons/Components/Viewport/NeuralNetwork.svg")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->DataElement(AZ::Edit::UIHandlers::Default, &MultilayerPerceptronComponent::m_model, "Model", "This is the machine-learning model provided by this component")
                    ;
            }
        }

        auto behaviorContext = azrtti_cast<AZ::BehaviorContext*>(context);
        if (behaviorContext)
        {
            behaviorContext->Class<MultilayerPerceptronComponent>("MultilayerPerceptron Component")->
                Attribute(AZ::Script::Attributes::Scope, AZ::Script::Attributes::ScopeFlags::Common)->
                Attribute(AZ::Script::Attributes::Module, "machineLearning")->
                Attribute(AZ::Script::Attributes::ExcludeFrom, AZ::Script::Attributes::ExcludeFlags::ListOnly)->
                Constructor<>()->
                Attribute(AZ::Script::Attributes::Storage, AZ::Script::Attributes::StorageType::Value)->
                Property("Model", BehaviorValueProperty(&MultilayerPerceptronComponent::m_model))
                ;

            behaviorContext->EBus<MultilayerPerceptronComponentRequestBus>("Multilayer perceptron requests")
                ->Attribute(AZ::Script::Attributes::Scope, AZ::Script::Attributes::ScopeFlags::Common)
                ->Attribute(AZ::Script::Attributes::Module, "machinelearning")
                ->Attribute(AZ::Script::Attributes::Category, "MachineLearning")
                ->Event("Get model", &MachineLearning::MultilayerPerceptronComponentRequestBus::Events::GetModel)
                ;
        }
    }

    void MultilayerPerceptronComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC("MultilayerPerceptronService"));
    }

    void MultilayerPerceptronComponent::Activate()
    {
        m_handle.reset(&m_model);
        MultilayerPerceptronComponentRequestBus::Handler::BusConnect(GetEntityId());
    }

    void MultilayerPerceptronComponent::Deactivate()
    {
        MultilayerPerceptronComponentRequestBus::Handler::BusDisconnect();
    }

    INeuralNetworkPtr MultilayerPerceptronComponent::GetModel()
    {
        return m_handle;
    }
}
