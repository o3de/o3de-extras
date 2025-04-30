/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzToolsFramework/API/ToolsApplicationAPI.h>

#include <AzToolsFramework/Entity/EditorEntityContextBus.h>
#include <Clients/ROS2SimulationInterfacesSystemComponent.h>
#include <ROS2/ROS2Bus.h>

namespace ROS2SimulationInterfaces
{
    /// System component for ROS2SimulationInterfaces editor
    class ROS2SimulationInterfacesEditorSystemComponent
        : public ROS2SimulationInterfacesSystemComponent
        , protected AzToolsFramework::EditorEvents::Bus::Handler
    {
        using BaseSystemComponent = ROS2SimulationInterfacesSystemComponent;

    public:
        AZ_COMPONENT_DECL(ROS2SimulationInterfacesEditorSystemComponent);

        static void Reflect(AZ::ReflectContext* context);

        ROS2SimulationInterfacesEditorSystemComponent();
        ~ROS2SimulationInterfacesEditorSystemComponent();

    private:
        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);

        // AZ::Component
        void Activate() override;
        void Deactivate() override;

        bool m_systemComponentActivated = false;
        ROS2::ROS2Requests::NodeChangedEvent::Handler m_nodeHandler;
    };
} // namespace ROS2SimulationInterfaces
