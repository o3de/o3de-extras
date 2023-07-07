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
#include <AzCore/RTTI/BehaviorContext.h>
#include <Models/Layer.h>
#include <Models/MultilayerPerceptron.h>
#include <AutoGenNodeableRegistry.generated.h>

static ScriptCanvas::MachineLearningPrivateObjectNodeableRegistry s_MachineLearningPrivateObjectNodeableRegistry;

namespace MachineLearning
{
    AZ_COMPONENT_IMPL(MachineLearningSystemComponent, "MachineLearningSystemComponent", MachineLearningSystemComponentTypeId);

    void MachineLearningSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<MachineLearningSystemComponent, AZ::Component>()->Version(0);
            serializeContext->Class<Layer>()->Version(0);
            serializeContext->Class<MultilayerPerceptron>()->Version(0);
        }

        if (auto behaviorContext = azrtti_cast<AZ::BehaviorContext*>(context))
        {
            behaviorContext->Class<MachineLearningSystemComponent>();
            behaviorContext->Class<Layer>();
            behaviorContext->Class<MultilayerPerceptron>();
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
