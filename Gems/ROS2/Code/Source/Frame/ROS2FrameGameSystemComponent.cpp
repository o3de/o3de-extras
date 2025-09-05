/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ROS2FrameGameSystemComponent.h"
#include <AzCore/Component/ComponentApplicationBus.h>
#include <AzCore/RTTI/BehaviorContext.h>
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
        if (auto bc = azrtti_cast<AZ::BehaviorContext*>(context))
        {
            bc->EBus<ROS2FrameTrackingRequestBus>("ROS2FrameTrackingRequestBus")
                ->Event("IsFrameRegistered", &ROS2FrameTrackingRequestBus::Events::IsFrameRegistered)
                ->Event("GetRegisteredFrameCount", &ROS2FrameTrackingRequestBus::Events::GetRegisteredFrameCount)
                ->Event("GetNamespacedFrameId", &ROS2FrameTrackingRequestBus::Events::GetNamespacedFrameId)
                ->Event("GetAllNamespacedFrameIds", &ROS2FrameTrackingRequestBus::Events::GetAllNamespacedFrameIds)
                ->Event("DisableEntitySafely", &ROS2FrameTrackingRequestBus::Events::DisableEntitySafely)
                ->Event("EnableEntitySafely", &ROS2FrameTrackingRequestBus::Events::EnableEntitySafely);
        }
    }

    void ROS2FrameGameSystemComponent::Activate()
    {
        ROS2FrameTrackingRequestBus::Handler::BusConnect();
    }

    void ROS2FrameGameSystemComponent::Deactivate()
    {
        ROS2FrameTrackingRequestBus::Handler::BusDisconnect();
        m_cachedChildTransforms.clear();
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

    const AZStd::unordered_set<AZ::EntityId>& ROS2FrameGameSystemComponent::GetRegisteredFrameEntityIds() const
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

    AZ::EntityId ROS2FrameGameSystemComponent::GetFrameEntityByNamespacedId(const AZStd::string& namespacedFrameId) const
    {
        auto it = m_frameIdToEntityId.find(namespacedFrameId);
        return (it != m_frameIdToEntityId.end()) ? it->second : AZ::EntityId();
    }

    AZStd::string ROS2FrameGameSystemComponent::GetNamespacedFrameId(const AZ::EntityId& frameEntityId) const
    {
        auto it = m_entityIdToFrameId.find(frameEntityId);
        return (it != m_entityIdToFrameId.end()) ? it->second : "";
    }

    AZStd::unordered_set<AZStd::string> ROS2FrameGameSystemComponent::GetAllNamespacedFrameIds() const
    {
        const auto valueView = AZStd::ranges::views::values(m_entityIdToFrameId);
        AZStd::unordered_set<AZStd::string> frameIds{ valueView.begin(), valueView.end() };
        return frameIds;
    }

    void ROS2FrameGameSystemComponent::DisableEntitySafely(const AZ::EntityId& entityToDisable)
    {
        // Disabling an entity is safe, but it cause children to be orphaned in the transform hierarchy.
        // That cause issues on reparenting in O3DE.
        // In this system component, we will cache local transform of childrens, so when the parent is re-enabled,
        AZ::Entity* entity = nullptr;
        AZ::ComponentApplicationBus::BroadcastResult(entity, &AZ::ComponentApplicationRequests::FindEntity, entityToDisable);
        AZ_Warning("ROS2FrameGameSystemComponent", entity, "Entity to disable not found, entity id: %s", entityToDisable.ToString().c_str());
        if (!entity)
        {
            return;
        }
        const auto children = entity->GetTransform()->GetChildren();
        AZStd::vector<EntityIdTransformPair> childTransforms;
        for (const auto& childId : children)
        {
            //get local transform of the child
            AZ::Transform childLocalTransform = AZ::Transform::CreateIdentity();
            AZ::TransformBus::EventResult(childLocalTransform, childId, &AZ::TransformBus::Events::GetLocalTM);
            childTransforms.emplace_back(EntityIdTransformPair{childId, childLocalTransform});
        }
        m_cachedChildTransforms.emplace(entityToDisable, childTransforms);
        entity->Deactivate();
    }

    void ROS2FrameGameSystemComponent::EnableEntitySafely(const AZ::EntityId& entityToEnable)
    {
        auto it = m_cachedChildTransforms.find(entityToEnable);
        if (it == m_cachedChildTransforms.end())
        {
            AZ_Warning("ROS2FrameGameSystemComponent", false, "EnableEntitySafely called on entity that was not disabled safely, entity id: %s", entityToEnable.ToString().c_str());
            return;
        }

        AZ::Entity* entity = nullptr;
        AZ::ComponentApplicationBus::BroadcastResult(entity, &AZ::ComponentApplicationRequests::FindEntity, entityToEnable);
        AZ_Warning("ROS2FrameGameSystemComponent", entity, "Entity to enable not found, entity id: %s", entityToEnable.ToString().c_str());
        if (!entity)
        {
            return;
        }
        entity->Activate();
        //restore local transform of the children
        for (const auto& [childId, childLocalTransform] : it->second)
        {
            AZ::TransformBus::Event(childId, &AZ::TransformBus::Events::SetLocalTM, childLocalTransform);
        }
        m_cachedChildTransforms.erase(it);
    }

} // namespace ROS2
