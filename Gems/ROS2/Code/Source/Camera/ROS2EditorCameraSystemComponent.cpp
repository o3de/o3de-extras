/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#include "ROS2EditorCameraSystemComponent.h"
#include <AzCore/Component/ComponentApplicationBus.h>
#include <AzCore/Serialization/SerializeContext.h>

namespace ROS2
{
    void ROS2EditorCameraSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<ROS2EditorCameraSystemComponent, BaseSystemComponent>()->Version(0);
        }
    }

    void ROS2EditorCameraSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        BaseSystemComponent::GetProvidedServices(provided);
        provided.push_back(AZ_CRC_CE("ROS2EditorCameraSystemService"));
    }

    void ROS2EditorCameraSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        BaseSystemComponent::GetIncompatibleServices(incompatible);
        incompatible.push_back(AZ_CRC_CE("ROS2EditorCameraSystemService"));
    }

    void ROS2EditorCameraSystemComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        BaseSystemComponent::GetRequiredServices(required);
    }

    void ROS2EditorCameraSystemComponent::Activate()
    {
        AzToolsFramework::EditorEntityContextNotificationBus::Handler::BusConnect();
        BaseSystemComponent::Activate();
    }

    void ROS2EditorCameraSystemComponent::Deactivate()
    {
        AzToolsFramework::EditorEntityContextNotificationBus::Handler::BusDisconnect();
    }

    void ROS2EditorCameraSystemComponent::OnStartPlayInEditorBegin()
    {
        BaseSystemComponent::Activate();
    }
    void ROS2EditorCameraSystemComponent::OnStopPlayInEditor()
    {
        BaseSystemComponent::Deactivate();
    }
} // namespace ROS2
