/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#include "WarehouseAutomationModuleInterface.h"
#include <AzCore/Memory/Memory.h>
#include <WarehouseAutomationSystemComponent.h>
#include <ConveyorBelt/ConveyorBeltComponent.h>
#include <ProximitySensor/ProximitySensor.h>
#include <WarehouseAutomation/WarehouseAutomationTypeIds.h>


namespace WarehouseAutomation
{
    AZ_TYPE_INFO_WITH_NAME_IMPL(WarehouseAutomationModuleInterface,
        "WarehouseAutomationModuleInterface", WarehouseAutomationModuleInterfaceTypeId);
    AZ_RTTI_NO_TYPE_INFO_IMPL(WarehouseAutomationModuleInterface, AZ::Module);
    AZ_CLASS_ALLOCATOR_IMPL(WarehouseAutomationModuleInterface, AZ::SystemAllocator);

    WarehouseAutomationModuleInterface::WarehouseAutomationModuleInterface()
    {
        // Push results of [MyComponent]::CreateDescriptor() into m_descriptors here.
        // Add ALL components descriptors associated with this gem to m_descriptors.
        // This will associate the AzTypeInfo information for the components with the the SerializeContext, BehaviorContext and EditContext.
        // This happens through the [MyComponent]::Reflect() function.
        m_descriptors.insert(m_descriptors.end(), {
            WarehouseAutomationSystemComponent::CreateDescriptor(),
            ConveyorBeltComponent::CreateDescriptor(),
            ProximitySensor::CreateDescriptor(),
            });
    }

    AZ::ComponentTypeList WarehouseAutomationModuleInterface::GetRequiredSystemComponents() const
    {
        return AZ::ComponentTypeList{
            azrtti_typeid<WarehouseAutomationSystemComponent>(),
        };
    }
} // namespace WarehouseAutomation
