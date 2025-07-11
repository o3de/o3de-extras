/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#include "ROS2EditorSystemComponent.h"
#include <AzCore/Serialization/SerializeContext.h>

#include <ROS2/ROS2TypeIds.h>

namespace ROS2
{
    AZ_COMPONENT_IMPL(ROS2EditorSystemComponent, "ROS2EditorSystemComponent", ROS2EditorSystemComponentTypeId, BaseSystemComponent);

    void ROS2EditorSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<ROS2EditorSystemComponent, ROS2SystemComponent>()->Version(0);
        }
    }

    ROS2EditorSystemComponent::ROS2EditorSystemComponent() = default;

    ROS2EditorSystemComponent::~ROS2EditorSystemComponent() = default;

    void ROS2EditorSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        BaseSystemComponent::GetProvidedServices(provided);
        provided.push_back(AZ_CRC_CE("ROS2EditorService"));
    }

    void ROS2EditorSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        BaseSystemComponent::GetIncompatibleServices(incompatible);
        incompatible.push_back(AZ_CRC_CE("ROS2EditorService"));
    }

    void ROS2EditorSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        BaseSystemComponent::GetRequiredServices(required);
    }

    void ROS2EditorSystemComponent::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
        BaseSystemComponent::GetDependentServices(dependent);
    }

    void ROS2EditorSystemComponent::Activate()
    {
        AzToolsFramework::EditorEntityContextNotificationBus::Handler::BusConnect();
        AzToolsFramework::EditorEvents::Bus::Handler::BusConnect();
    }

    void ROS2EditorSystemComponent::Deactivate()
    {
        AzToolsFramework::EditorEvents::Bus::Handler::BusDisconnect();
        AzToolsFramework::EditorEntityContextNotificationBus::Handler::BusDisconnect();
    }

    void ROS2EditorSystemComponent::OnStartPlayInEditorBegin()
    {
        ROS2SystemComponent::Activate();
    }
    void ROS2EditorSystemComponent::OnStopPlayInEditor()
    {
        ROS2SystemComponent::Deactivate();
    }

} // namespace ROS2
