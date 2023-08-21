/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzToolsFramework/API/ToolsApplicationAPI.h>

#include <WarehouseAutomationSystemComponent.h>

namespace WarehouseAutomation
{
    /// System component for WarehouseAutomation editor
    class WarehouseAutomationEditorSystemComponent
        : public WarehouseAutomationSystemComponent
        , protected AzToolsFramework::EditorEvents::Bus::Handler
    {
        using BaseSystemComponent = WarehouseAutomationSystemComponent;
    public:
        AZ_COMPONENT_DECL(WarehouseAutomationEditorSystemComponent);

        static void Reflect(AZ::ReflectContext* context);

        WarehouseAutomationEditorSystemComponent();
        ~WarehouseAutomationEditorSystemComponent();

    private:
        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);

        // AZ::Component
        void Activate() override;
        void Deactivate() override;
    };
} // namespace WarehouseAutomation
