/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzToolsFramework/API/ToolsApplicationAPI.h>

#include <Clients/SimulationInterfacesROS2SystemComponent.h>

namespace SimulationInterfacesROS2
{
    /// System component for SimulationInterfacesROS2 editor
    class SimulationInterfacesROS2EditorSystemComponent
        : public SimulationInterfacesROS2SystemComponent
        , protected AzToolsFramework::EditorEvents::Bus::Handler
    {
        using BaseSystemComponent = SimulationInterfacesROS2SystemComponent;

    public:
        AZ_COMPONENT_DECL(SimulationInterfacesROS2EditorSystemComponent);

        static void Reflect(AZ::ReflectContext* context);

        SimulationInterfacesROS2EditorSystemComponent();
        ~SimulationInterfacesROS2EditorSystemComponent();

    private:
        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);

        // AZ::Component
        void Activate() override;
        void Deactivate() override;
    };
} // namespace SimulationInterfacesROS2
