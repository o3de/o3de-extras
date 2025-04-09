/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root
 * of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzToolsFramework/API/ToolsApplicationAPI.h>

#include <Clients/ROS2ControllersSystemComponent.h>

namespace ROS2Controllers
{
    /// System component for ROS2Controllers editor
    class ROS2ControllersEditorSystemComponent
        : public ROS2ControllersSystemComponent
        , protected AzToolsFramework::EditorEvents::Bus::Handler
    {
        using BaseSystemComponent = ROS2ControllersSystemComponent;

    public:
        AZ_COMPONENT_DECL(ROS2ControllersEditorSystemComponent);

        static void Reflect(AZ::ReflectContext* context);

        ROS2ControllersEditorSystemComponent();
        ~ROS2ControllersEditorSystemComponent();

    private:
        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);

        // AZ::Component
        void Activate() override;
        void Deactivate() override;
    };
} // namespace ROS2Controllers
