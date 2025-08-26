/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ROS2FrameSystemComponent.h"
#include <AzCore/Component/TickBus.h>
#include <AzCore/RTTI/BehaviorContext.h>
#include <AzCore/Serialization/SerializeContext.h>

namespace ROS2
{
    void ROS2FrameSystemComponent::Activate()
    {
        ROS2FrameRegistrationInterface::Register(this);
        ROS2FrameTrackingInterface::Register(this);
    }

    void ROS2FrameSystemComponent::Deactivate()
    {
        ROS2FrameTrackingInterface::Unregister(this);
        ROS2FrameRegistrationInterface::Unregister(this);
        m_registeredFrames.clear();
        m_namespacedFrameIdToEntity.clear();
        m_entityToNamespacedFrameId.clear();
    }

    void ROS2FrameSystemComponent::RegisterFrame(const AZ::EntityId& frameEntityId)
    {
        if (frameEntityId.IsValid())
        {
            m_registeredFrames.insert(frameEntityId);

            AZStd::string namespacedFrameId;
            ROS2FrameComponentBus::EventResult(namespacedFrameId, frameEntityId, &ROS2FrameComponentBus::Events::GetNamespacedFrameID);
            
            if (!namespacedFrameId.empty())
            {
                m_namespacedFrameIdToEntity[namespacedFrameId] = frameEntityId;
                m_entityToNamespacedFrameId[frameEntityId] = namespacedFrameId;
            }
        }
    }

    void ROS2FrameSystemComponent::UnregisterFrame(const AZ::EntityId& frameEntityId)
    {
        m_registeredFrames.erase(frameEntityId);

        auto entityToNamespacedIt = m_entityToNamespacedFrameId.find(frameEntityId);
        if (entityToNamespacedIt != m_entityToNamespacedFrameId.end())
        {
            const AZStd::string& namespacedFrameId = entityToNamespacedIt->second;
            m_namespacedFrameIdToEntity.erase(namespacedFrameId);
            m_entityToNamespacedFrameId.erase(entityToNamespacedIt);
        }
    }

    const AZStd::unordered_set<AZ::EntityId>& ROS2FrameSystemComponent::GetRegisteredFrames() const
    {
        return m_registeredFrames;
    }

    bool ROS2FrameSystemComponent::IsFrameRegistered(const AZ::EntityId& frameEntityId) const
    {
        return m_registeredFrames.find(frameEntityId) != m_registeredFrames.end();
    }

    size_t ROS2FrameSystemComponent::GetRegisteredFrameCount() const
    {
        return m_registeredFrames.size();
    }

    AZ::EntityId ROS2FrameSystemComponent::GetFrameEntityByNamespacedId(const AZStd::string& namespacedFrameId) const
    {
        auto it = m_namespacedFrameIdToEntity.find(namespacedFrameId);
        return it != m_namespacedFrameIdToEntity.end() ? it->second : AZ::EntityId();
    }

    AZStd::string ROS2FrameSystemComponent::GetNamespacedFrameId(const AZ::EntityId& frameEntityId) const
    {
        auto it = m_entityToNamespacedFrameId.find(frameEntityId);
        return it != m_entityToNamespacedFrameId.end() ? it->second : AZStd::string();
    }

    AZStd::unordered_set<AZStd::string> ROS2FrameSystemComponent::GetAllNamespacedFrameIds() const
    {
        AZStd::unordered_set<AZStd::string> namespacedFrameIds;
        for (const auto& pair : m_namespacedFrameIdToEntity)
        {
            namespacedFrameIds.insert(pair.first);
        }
        return namespacedFrameIds;
    }

    void ROS2FrameSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<ROS2FrameSystemComponent, AZ::Component>()
                ->Version(1);
        }
    }
} // namespace ROS2
