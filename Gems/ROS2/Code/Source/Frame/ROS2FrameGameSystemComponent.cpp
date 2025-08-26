/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ROS2FrameGameSystemComponent.h"
#include <AzCore/Component/ComponentApplicationBus.h>
#include <AzCore/std/ranges/elements_view.h>
#include <ROS2/Frame/ROS2FrameComponent.h>
#include <ROS2/Frame/ROS2FrameComponentBus.h>

namespace ROS2
{

    ROS2FrameGameSystemComponent::ROS2FrameGameSystemComponent()
    {
        if (ROS2FrameSystemInterface::Get() == nullptr)
        {
            ROS2FrameSystemInterface::Register(this);
        }
        if (ROS2FrameTrackingInterface::Get() == nullptr)
        {
            ROS2FrameTrackingInterface::Register(this);
        }
    }

    ROS2FrameGameSystemComponent::~ROS2FrameGameSystemComponent()
    {
        if (ROS2FrameSystemInterface::Get() == this)
        {
            ROS2FrameSystemInterface::Unregister(this);
        }
        if (ROS2FrameTrackingInterface::Get() == this)
        {
            ROS2FrameTrackingInterface::Unregister(this);
        }
    }

    void ROS2FrameGameSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<ROS2FrameGameSystemComponent, AZ::Component>()->Version(1);
        }
    }

    void ROS2FrameGameSystemComponent::Activate()
    {
    }

    void ROS2FrameGameSystemComponent::Deactivate()
    {
        m_registeredEntities.clear();
        m_entityIdToFrameId.clear();
        m_frameIdToEntityId.clear();
    }

    void ROS2FrameGameSystemComponent::RegisterFrame(const AZ::EntityId& frameEntityId)
    {
        m_registeredEntities.insert(frameEntityId);

        AZStd::string namespaceFrameId = "";
        ROS2FrameComponentBus::EventResult(namespaceFrameId, frameEntityId, &ROS2FrameComponentBus::Events::GetNamespacedFrameID);
        if (!namespaceFrameId.empty())
        {
            m_frameIdToEntityId[namespaceFrameId] = frameEntityId;
            m_entityIdToFrameId[frameEntityId] = namespaceFrameId;
        }
    }

    void ROS2FrameGameSystemComponent::UnregisterFrame(const AZ::EntityId& frameEntityId)
    {
        m_registeredEntities.erase(frameEntityId);
        AZStd::string namespacedFrameId = m_entityIdToFrameId[frameEntityId];
        m_frameIdToEntityId.erase(namespacedFrameId);
        m_entityIdToFrameId.erase(frameEntityId);
    }

    const AZStd::unordered_set<AZ::EntityId>& ROS2FrameGameSystemComponent::GetRegisteredFrames() const
    {
        return m_registeredEntities;
    }

    bool ROS2FrameGameSystemComponent::IsFrameRegistered(const AZ::EntityId& frameEntityId) const
    {
        return m_registeredEntities.contains(frameEntityId);
    }

    size_t ROS2FrameGameSystemComponent::GetRegisteredFrameCount() const
    {
        return m_registeredEntities.size();
    }

    AZStd::optional<AZ::EntityId> ROS2FrameGameSystemComponent::GetFrameEntityByNamespacedId(const AZStd::string& namespacedFrameId) const
    {
        auto it = m_frameIdToEntityId.find(namespacedFrameId);
        return (it != m_frameIdToEntityId.end()) ? AZStd::optional<AZ::EntityId>(it->second) : AZStd::nullopt;
    }

    AZStd::optional<AZStd::string> ROS2FrameGameSystemComponent::GetNamespacedFrameId(const AZ::EntityId& frameEntityId) const
    {
        auto it = m_entityIdToFrameId.find(frameEntityId);
        return (it != m_entityIdToFrameId.end()) ? AZStd::optional<AZStd::string>(it->second) : AZStd::nullopt;
    }

    AZStd::unordered_set<AZStd::string> ROS2FrameGameSystemComponent::GetAllNamespacedFrameIds() const
    {
        const auto valueView = AZStd::ranges::views::values(m_entityIdToFrameId);
        AZStd::unordered_set<AZStd::string> frameIds{ valueView.begin(), valueView.end() };
        return frameIds;
    }

} // namespace ROS2
