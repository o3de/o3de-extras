/*
* Copyright (c) Contributors to the Open 3D Engine Project.
* For complete copyright and license terms please see the LICENSE at the root of this distribution.
*
* SPDX-License-Identifier: Apache-2.0 OR MIT
*
*/

#pragma once

#include <AzToolsFramework/API/ToolsApplicationAPI.h>

#include <Clients/GeoreferencingSystemComponent.h>

namespace Georeferencing
{
    /// System component for Georeferencing editor
    class GeoreferencingEditorSystemComponent
        : public GeoreferencingSystemComponent
        , protected AzToolsFramework::EditorEvents::Bus::Handler
    {
        using BaseSystemComponent = GeoreferencingSystemComponent;

    public:
        AZ_COMPONENT_DECL(GeoreferencingEditorSystemComponent);

        static void Reflect(AZ::ReflectContext* context);

        GeoreferencingEditorSystemComponent() = default;
        ~GeoreferencingEditorSystemComponent() override = default;

    private:
        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);

        // AZ::Component
        void Activate() override;
        void Deactivate() override;
    };
} // namespace Georeferencing
