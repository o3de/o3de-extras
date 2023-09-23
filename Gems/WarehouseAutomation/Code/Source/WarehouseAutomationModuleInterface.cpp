/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#include "WarehouseAutomationModuleInterface.h"
#include <AzCore/Memory/Memory.h>
#include <ConveyorBelt/ConveyorBeltComponent.h>
#include <ProximitySensor/ProximitySensor.h>

namespace WarehouseAutomation
{
    AZ_TYPE_INFO_WITH_NAME_IMPL(
        WarehouseAutomationModuleInterface, "WarehouseAutomationModuleInterface", "{6584FD8A-0FDA-48CD-A2E6-43F08CC9407E}");
    AZ_RTTI_NO_TYPE_INFO_IMPL(WarehouseAutomationModuleInterface, AZ::Module);
    AZ_CLASS_ALLOCATOR_IMPL(WarehouseAutomationModuleInterface, AZ::SystemAllocator);

    WarehouseAutomationModuleInterface::WarehouseAutomationModuleInterface()
    {
        m_descriptors.insert(
            m_descriptors.end(),
            {
                ConveyorBeltComponent::CreateDescriptor(),
                ProximitySensor::CreateDescriptor(),
            });
    }
} // namespace WarehouseAutomation
