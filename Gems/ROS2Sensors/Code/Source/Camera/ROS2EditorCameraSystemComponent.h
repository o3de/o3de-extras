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
#include <ROS2Sensors/ROS2SensorsTypeIds.h>

namespace ROS2
{
    /// Editor system component for ROS2 camera simulation.
    class ROS2EditorCameraSystemComponent
        : public ROS2SystemCameraComponent
        , private AzToolsFramework::EditorEntityContextNotificationBus::Handler
    {
        using BaseSystemComponent = ROS2SystemCameraComponent;

    public:
        AZ_COMPONENT(ROS2EditorCameraSystemComponent, ROS2Sensors::ROS2EditorCameraSystemComponentTypeId, BaseSystemComponent);
        static void Reflect(AZ::ReflectContext* context);

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
