/*
* Copyright (c) Contributors to the Open 3D Engine Project.
* For complete copyright and license terms please see the LICENSE at the root of this distribution.
*
* SPDX-License-Identifier: Apache-2.0 OR MIT
*
*/

#include "GeoreferencingEditorSystemComponent.h"
#include <AzCore/Serialization/SerializeContext.h>

#include <Georeferencing/GeoreferencingTypeIds.h>

namespace Georeferencing
{
    AZ_COMPONENT_IMPL(
        GeoreferencingEditorSystemComponent,
        "GeoreferencingEditorSystemComponent",
        GeoreferencingEditorSystemComponentTypeId,
        BaseSystemComponent);

    void GeoreferencingEditorSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<GeoreferencingEditorSystemComponent, GeoreferencingSystemComponent>()->Version(0);
        }
    }

    void GeoreferencingEditorSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        BaseSystemComponent::GetProvidedServices(provided);
        provided.push_back(AZ_CRC_CE("GeoreferencingEditorService"));
    }

    void GeoreferencingEditorSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        BaseSystemComponent::GetIncompatibleServices(incompatible);
        incompatible.push_back(AZ_CRC_CE("GeoreferencingEditorService"));
    }

    void GeoreferencingEditorSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        BaseSystemComponent::GetRequiredServices(required);
    }

    void GeoreferencingEditorSystemComponent::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
        BaseSystemComponent::GetDependentServices(dependent);
    }

    void GeoreferencingEditorSystemComponent::Activate()
    {
        GeoreferencingSystemComponent::Activate();
        AzToolsFramework::EditorEvents::Bus::Handler::BusConnect();
    }

    void GeoreferencingEditorSystemComponent::Deactivate()
    {
        AzToolsFramework::EditorEvents::Bus::Handler::BusDisconnect();
        GeoreferencingSystemComponent::Deactivate();
    }

} // namespace Georeferencing
