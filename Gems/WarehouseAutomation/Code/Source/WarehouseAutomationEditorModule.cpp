/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#include <WarehouseAutomationModuleInterface.h>

namespace WarehouseAutomation
{
    class WarehouseAutomationEditorModule : public WarehouseAutomationModuleInterface
    {
    public:
        AZ_RTTI(WarehouseAutomationEditorModule, "{906B71F8-374D-4F8E-B8F2-EAFEFF863F7F}", WarehouseAutomationModuleInterface);
        AZ_CLASS_ALLOCATOR(WarehouseAutomationEditorModule, AZ::SystemAllocator);

        WarehouseAutomationEditorModule()
        {
            m_descriptors.insert(m_descriptors.end(), {});
        }
    };
} // namespace WarehouseAutomation

AZ_DECLARE_MODULE_CLASS(Gem_WarehouseAutomation, WarehouseAutomation::WarehouseAutomationEditorModule)
