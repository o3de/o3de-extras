/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ROS2FrameEditorSystemComponent.h"
#include "NamespaceComputation.h"
#include "ROS2FrameEditorComponent.h"
#include "ROS2FrameSystemBus.h"

#include <AzCore/Component/ComponentApplicationBus.h>
#include <AzCore/Component/ComponentBus.h>
#include <AzCore/Component/Entity.h>
#include <AzCore/Component/EntityId.h>
#include <AzCore/std/containers/set.h>
#include <AzCore/std/containers/vector.h>
#include <AzCore/std/string/string.h>
#include <AzToolsFramework/ToolsComponents/TransformComponent.h>
#include <ROS2/Frame/ROS2FrameComponent.h>
#include <ROS2/Frame/ROS2FrameEditorComponentBus.h>
#include <ROS2/ROS2NamesBus.h>

namespace ROS2
{

    ROS2FrameEditorSystemComponent::ROS2FrameEditorSystemComponent()
        : ROS2FrameGameSystemComponent()
    {
    }

    void ROS2FrameEditorSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<ROS2FrameEditorSystemComponent, AZ::Component>()->Version(1)->Attribute(
                AZ::Edit::Attributes::SystemComponentTags, AZStd::vector<AZ::Crc32>({ AZ_CRC_CE("AssetBuilder") }));
        }
    }

    void ROS2FrameEditorSystemComponent::Activate()
    {
        ROS2FrameGameSystemComponent::Activate();
    }

    void ROS2FrameEditorSystemComponent::Deactivate()
    {
        for (const auto& id : m_registeredEntities)
        {
            AZ::TransformNotificationBus::MultiHandler::BusDisconnect(id);
            AzToolsFramework::EntitySelectionEvents::Bus::MultiHandler::BusDisconnect(id);
        }

        ROS2FrameGameSystemComponent::Deactivate();
    }

    void ROS2FrameEditorSystemComponent::RegisterFrame(const AZ::EntityId& frameToRegister)
    {
        ROS2FrameGameSystemComponent::RegisterFrame(frameToRegister);

        AzToolsFramework::EntitySelectionEvents::Bus::MultiHandler::BusConnect(frameToRegister);
        AZ::TransformNotificationBus::MultiHandler::BusConnect(frameToRegister);
    }

    void ROS2FrameEditorSystemComponent::UnregisterFrame(const AZ::EntityId& frameToUnregister)
    {
        AZ::TransformNotificationBus::MultiHandler::BusDisconnect(frameToUnregister);
        AzToolsFramework::EntitySelectionEvents::Bus::MultiHandler::BusDisconnect(frameToUnregister);

        ROS2FrameGameSystemComponent::UnregisterFrame(frameToUnregister);
    }

    void ROS2FrameEditorSystemComponent::OnSelected()
    {
        // find which frame entity was selected
        AZStd::vector<AZ::EntityId> selectedEntityId;
        AzToolsFramework::ToolsApplicationRequests::Bus::BroadcastResult(
            selectedEntityId, &AzToolsFramework::ToolsApplicationRequests::GetSelectedEntities);
        // update
        for (const auto& selectedEntityId : selectedEntityId)
        {
            ROS2FrameEditorComponentBus::Event(selectedEntityId, &ROS2FrameEditorComponentRequests::UpdateNamespace);
        }
    }

    void ROS2FrameEditorSystemComponent::OnParentChanged([[maybe_unused]] AZ::EntityId oldParent, AZ::EntityId newParent)
    {
        AZStd::vector<AZ::EntityId> children;
        AZ::TransformBus::EventResult(children, newParent, &AZ::TransformBus::Events::GetEntityAndAllDescendants);
        // update
        for (const auto& child : children)
        {
            ROS2FrameEditorComponentBus::Event(child, &ROS2FrameEditorComponentRequests::UpdateNamespace);
        }
    }

} // namespace ROS2
