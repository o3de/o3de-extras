/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 */

#include "O3DEReflectSystemComponent.h"

#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>
#include <AzCore/RTTI/BehaviorContext.h>

namespace O3DEReflect
{
    AZ_COMPONENT_IMPL(O3DEReflectSystemComponent, "O3DEReflectSystemComponent",
        "{F8E7A3B2-5C4D-4E6F-8A9B-1C2D3E4F5A6B}");

    void O3DEReflectSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<O3DEReflectSystemComponent, AZ::Component>()
                ->Version(1);

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<O3DEReflectSystemComponent>("O3DE Reflect System", 
                    "Provides runtime support for the O3DE Reflect code generation system")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("System"))
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true);
            }
        }

        if (auto behaviorContext = azrtti_cast<AZ::BehaviorContext*>(context))
        {
            behaviorContext->Class<O3DEReflectSystemComponent>("O3DEReflectSystem")
                ->Attribute(AZ::Script::Attributes::Category, "O3DEReflect");
        }
    }

    void O3DEReflectSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("O3DEReflectService"));
    }

    void O3DEReflectSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("O3DEReflectService"));
    }

    void O3DEReflectSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
    }

    void O3DEReflectSystemComponent::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
    }

    void O3DEReflectSystemComponent::Init()
    {
    }

    void O3DEReflectSystemComponent::Activate()
    {
    }

    void O3DEReflectSystemComponent::Deactivate()
    {
    }

} // namespace O3DEReflect
