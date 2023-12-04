/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include "ROS2CameraSystemComponent.h"

#include <AzToolsFramework/Entity/EditorEntityContextBus.h>

namespace ROS2
{
    /// System component for ROS2 editor
    class ROS2EditorCameraSystemComponent
        : public ROS2SystemCameraComponent
        , private AzToolsFramework::EditorEntityContextNotificationBus::Handler
    {
        using BaseSystemComponent = ROS2SystemCameraComponent;

    public:
        AZ_COMPONENT(ROS2EditorCameraSystemComponent, "{407f51c0-92c9-11ee-b9d1-0242ac120002}", BaseSystemComponent);
        static void Reflect(AZ::ReflectContext* context);

        ROS2EditorCameraSystemComponent();
        ~ROS2EditorCameraSystemComponent();

    private:
        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);

        //////////////////////////////////////////////////////////////////////////
        // Component overrides
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
