/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#include <WarehouseAutomation/WarehouseAutomationTypeIds.h>
#include <WarehouseAutomationModuleInterface.h>
#include "WarehouseAutomationSystemComponent.h"

namespace WarehouseAutomation
{
    class WarehouseAutomationModule
        : public WarehouseAutomationModuleInterface
    {
    public:
        AZ_RTTI(WarehouseAutomationModule, WarehouseAutomationModuleTypeId, WarehouseAutomationModuleInterface);
        AZ_CLASS_ALLOCATOR(WarehouseAutomationModule, AZ::SystemAllocator);
    };
}// namespace WarehouseAutomation

AZ_DECLARE_MODULE_CLASS(Gem_WarehouseAutomation, WarehouseAutomation::WarehouseAutomationModule)
