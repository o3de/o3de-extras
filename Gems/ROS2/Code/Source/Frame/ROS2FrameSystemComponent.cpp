/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ROS2FrameSystemComponent.h"
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/RTTI/BehaviorContext.h>
#include <AzCore/Component/TickBus.h>
#include <iostream>

namespace ROS2
{
    void ROS2FrameSystemComponent::Activate()
    {

        std::cout << "Activating ROS2FrameSystemComponent" << std::endl<< std::endl<< std::endl<< std::endl<< std::endl<< std::endl<< std::endl<< std::endl;
        ROS2FrameRegistrationInterface::Register(this);
        ROS2FrameTrackingInterface::Register(this);

        // Connect to SystemTickBus for periodic debug printing
        m_tickCounter = 0;
        AZ::SystemTickBus::Handler::BusConnect();
    }

    void ROS2FrameSystemComponent::Deactivate()
    {
        AZ::SystemTickBus::Handler::BusDisconnect();
        ROS2FrameTrackingInterface::Unregister(this);
        ROS2FrameRegistrationInterface::Unregister(this);
        m_registeredFrames.clear();
        m_namespacedFrameIdToEntity.clear();
        m_entityToNamespacedFrameId.clear();
    }

    void ROS2FrameSystemComponent::OnSystemTick()
    {
        m_tickCounter++;
        if (m_tickCounter >= 60)
        {
            DebugPrintMaps();
            m_tickCounter = 0;
        }
    }

    void ROS2FrameSystemComponent::RegisterFrame(const AZ::EntityId& frameEntityId)
    {
        if (frameEntityId.IsValid())
        {
            m_registeredFrames.insert(frameEntityId);
            
            // Query the frame component for its namespaced frame ID
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
        
        // Remove from namespaced frame ID mappings
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

        if (auto behaviorContext = azrtti_cast<AZ::BehaviorContext*>(context))
        {
            behaviorContext->EBus<ROS2FrameRegistrationBus>("ROS2FrameRegistrationBus")
                ->Attribute(AZ::Script::Attributes::Scope, AZ::Script::Attributes::ScopeFlags::Common)
                ->Attribute(AZ::Script::Attributes::Category, "ROS2")
                ->Event("RegisterFrame", &ROS2FrameRegistrationBus::Events::RegisterFrame)
                ->Event("UnregisterFrame", &ROS2FrameRegistrationBus::Events::UnregisterFrame);

            behaviorContext->EBus<ROS2FrameTrackingBus>("ROS2FrameTrackingBus")
                ->Attribute(AZ::Script::Attributes::Scope, AZ::Script::Attributes::ScopeFlags::Common)
                ->Attribute(AZ::Script::Attributes::Category, "ROS2")
                ->Event("GetRegisteredFrames", &ROS2FrameTrackingBus::Events::GetRegisteredFrames)
                ->Event("IsFrameRegistered", &ROS2FrameTrackingBus::Events::IsFrameRegistered)
                ->Event("GetRegisteredFrameCount", &ROS2FrameTrackingBus::Events::GetRegisteredFrameCount)
                ->Event("GetFrameEntityByNamespacedId", &ROS2FrameTrackingBus::Events::GetFrameEntityByNamespacedId)
                ->Event("GetNamespacedFrameId", &ROS2FrameTrackingBus::Events::GetNamespacedFrameId)
                ->Event("GetAllNamespacedFrameIds", &ROS2FrameTrackingBus::Events::GetAllNamespacedFrameIds);
        }
    }

    void ROS2FrameSystemComponent::DebugPrintMaps() const
    {
        std::cout << "=== Debug Frame Maps ===" << std::endl;
        
        std::cout << "Registered Frames (" << m_registeredFrames.size() << "):" << std::endl;
        for (const auto& entityId : m_registeredFrames)
        {
            std::cout << "  - EntityId: " << entityId.ToString().c_str() << std::endl;
        }
        
        std::cout << "Namespaced Frame ID to Entity Map (" << m_namespacedFrameIdToEntity.size() << "):" << std::endl;
        for (const auto& pair : m_namespacedFrameIdToEntity)
        {
            std::cout << "  - '" << pair.first.c_str() << "' -> " << pair.second.ToString().c_str() << std::endl;
        }
        
        std::cout << "Entity to Namespaced Frame ID Map (" << m_entityToNamespacedFrameId.size() << "):" << std::endl;
        for (const auto& pair : m_entityToNamespacedFrameId)
        {
            std::cout << "  - " << pair.first.ToString().c_str() << " -> '" << pair.second.c_str() << "'" << std::endl;
        }
        
        std::cout << "=== End Debug Frame Maps ===" << std::endl;
    }
} // namespace ROS2
