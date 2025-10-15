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
#include <Clock/ROS2ClockSystemComponent.h>

namespace ROS2
{
    /// System component for ROS2 editor
    class ROS2EditorClockSystemComponent
        : public ROS2ClockSystemComponent
        , protected AzToolsFramework::EditorEvents::Bus::Handler
        , private AzToolsFramework::EditorEntityContextNotificationBus::Handler
    {
        using BaseSystemComponent = ROS2ClockSystemComponent;

    public:
        AZ_COMPONENT_DECL(ROS2EditorClockSystemComponent);

        static void Reflect(AZ::ReflectContext* context);

        ROS2EditorClockSystemComponent() = default;
        ~ROS2EditorClockSystemComponent() override = default;

    private:
        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);

        //////////////////////////////////////////////////////////////////////////
        // AZ::Component overrides
        void Activate() override;
        void Deactivate() override;
        //////////////////////////////////////////////////////////////////////////

        //////////////////////////////////////////////////////////////////////////
        // EditorEntityContextNotificationBus overrides
        void OnStartPlayInEditorBegin() override;
        void OnStopPlayInEditor() override;
        //////////////////////////////////////////////////////////////////////////
    };
} // namespace ROS2
