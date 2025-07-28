/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#include "ROS2EditorClockSystemComponent.h"
#include <AzCore/Serialization/SerializeContext.h>

#include <ROS2/ROS2TypeIds.h>

namespace ROS2
{
    AZ_COMPONENT_IMPL(ROS2EditorClockSystemComponent, "ROS2EditorClockSystemComponent", ROS2EditorClockSystemComponentTypeId, BaseSystemComponent);

    void ROS2EditorClockSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<ROS2EditorSystemComponent, ROS2SystemComponent>()->Version(0);
        }
    }

    void ROS2EditorClockSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        BaseSystemComponent::GetProvidedServices(provided);
        provided.push_back(AZ_CRC_CE("ROS2EditorClockService"));
    }

    void ROS2EditorClockSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        BaseSystemComponent::GetIncompatibleServices(incompatible);
        incompatible.push_back(AZ_CRC_CE("ROS2EditorClockService"));
    }

    void ROS2EditorClockSystemComponent::Activate()
    {
        AzToolsFramework::EditorEntityContextNotificationBus::Handler::BusConnect();
        AzToolsFramework::EditorEvents::Bus::Handler::BusConnect();
    }

    void ROS2EditorClockSystemComponent::Deactivate()
    {
        AzToolsFramework::EditorEvents::Bus::Handler::BusDisconnect();
        AzToolsFramework::EditorEntityContextNotificationBus::Handler::BusDisconnect();
    }

    void ROS2EditorClockSystemComponent::OnStartPlayInEditorBegin()
    {
        BaseSystemComponent::Activate();
    }
    void ROS2EditorClockSystemComponent::OnStopPlayInEditor()
    {
        BaseSystemComponent::Deactivate();
    }

} // namespace ROS2
