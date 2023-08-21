/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#include <AzCore/Serialization/SerializeContext.h>
#include "WarehouseAutomationEditorSystemComponent.h"

#include <WarehouseAutomation/WarehouseAutomationTypeIds.h>

namespace WarehouseAutomation
{
    AZ_COMPONENT_IMPL(WarehouseAutomationEditorSystemComponent, "WarehouseAutomationEditorSystemComponent",
        WarehouseAutomationEditorSystemComponentTypeId, BaseSystemComponent);

    void WarehouseAutomationEditorSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<WarehouseAutomationEditorSystemComponent, WarehouseAutomationSystemComponent>()
                ->Version(0);
        }
    }

    WarehouseAutomationEditorSystemComponent::WarehouseAutomationEditorSystemComponent() = default;

    WarehouseAutomationEditorSystemComponent::~WarehouseAutomationEditorSystemComponent() = default;

    void WarehouseAutomationEditorSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        BaseSystemComponent::GetProvidedServices(provided);
        provided.push_back(AZ_CRC_CE("WarehouseAutomationEditorService"));
    }

    void WarehouseAutomationEditorSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        BaseSystemComponent::GetIncompatibleServices(incompatible);
        incompatible.push_back(AZ_CRC_CE("WarehouseAutomationEditorService"));
    }

    void WarehouseAutomationEditorSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        BaseSystemComponent::GetRequiredServices(required);
    }

    void WarehouseAutomationEditorSystemComponent::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
        BaseSystemComponent::GetDependentServices(dependent);
    }

    void WarehouseAutomationEditorSystemComponent::Activate()
    {
        WarehouseAutomationSystemComponent::Activate();
        AzToolsFramework::EditorEvents::Bus::Handler::BusConnect();
    }

    void WarehouseAutomationEditorSystemComponent::Deactivate()
    {
        AzToolsFramework::EditorEvents::Bus::Handler::BusDisconnect();
        WarehouseAutomationSystemComponent::Deactivate();
    }

} // namespace WarehouseAutomation
