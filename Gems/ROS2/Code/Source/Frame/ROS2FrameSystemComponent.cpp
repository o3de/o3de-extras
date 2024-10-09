/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ROS2FrameSystemComponent.h"
#include "AzCore/std/containers/vector.h"
#include "ROS2FrameSystemBus.h"
#include <AzCore/Component/ComponentApplicationBus.h>
#include <AzCore/Component/ComponentBus.h>
#include <AzCore/Component/Entity.h>
#include <AzCore/Component/EntityId.h>
#include <AzCore/std/containers/set.h>
#include <AzCore/std/string/string.h>
#include <AzToolsFramework/ToolsComponents/TransformComponent.h>
#include <ROS2/Frame/ROS2FrameBus.h>
#include <ROS2/Frame/ROS2FrameComponent.h>
#include <ROS2/Frame/ROS2FrameEditorComponent.h>
#include <ROS2/ROS2GemUtilities.h>

namespace ROS2
{

    void ROS2FrameSystemTransformHandler::OnParentChanged(AZ::EntityId oldParent, AZ::EntityId newParent)
    {
        for (auto frameEntityId : m_frameEntities)
        {
            ROS2FrameSystemInterface::Get()->MoveFrame(frameEntityId, newParent);
        }
    }

    void ROS2FrameSystemTransformHandler::AddFrameEntity(AZ::EntityId frameEntityId)
    {
        if (!m_frameEntities.contains(frameEntityId))
        {
            m_frameEntities.insert(frameEntityId);
        }
    }
    void ROS2FrameSystemTransformHandler::RemoveFrameEntity(AZ::EntityId frameEntityId)
    {
        if (m_frameEntities.contains(frameEntityId))
        {
            m_frameEntities.erase(frameEntityId);
        }
    }

    unsigned int ROS2FrameSystemTransformHandler::GetFrameCount()
    {
        return m_frameEntities.size();
    }

    ROS2FrameSystemComponent::ROS2FrameSystemComponent()
    {
        if (ROS2FrameSystemInterface::Get() == nullptr)
        {
            ROS2FrameSystemInterface::Register(this);
        }
    }

    ROS2FrameSystemComponent::~ROS2FrameSystemComponent()
    {
        if (ROS2FrameSystemInterface::Get() == this)
        {
            ROS2FrameSystemInterface::Unregister(this);
        }
    }

    void ROS2FrameSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<ROS2FrameSystemComponent, AZ::Component>();
        }
    }

    void ROS2FrameSystemComponent::Activate()
    {
    }

    void ROS2FrameSystemComponent::Deactivate()
    {
    }

    AZ::TransformInterface* ROS2FrameSystemComponent::GetEntityTransformInterface(const AZ::Entity* entity)
    {
        if (!entity)
        {
            AZ_Error("GetEntityTransformInterface", false, "Invalid entity!");
            return nullptr;
        }

        auto* interface = Utils::GetGameOrEditorComponent<AzToolsFramework::Components::TransformComponent>(entity);

        return interface;
    }

    bool ROS2FrameSystemComponent::IsTopLevel(const AZ::EntityId& frameEntityId) const
    {
        if (m_frameParent.contains(frameEntityId))
        {
            const AZ::EntityId& parent = m_frameParent.find(frameEntityId)->second;
            return m_frameParent.find(parent)->second == parent;
        }

        return false;
    }

    AZ::EntityId ROS2FrameSystemComponent::GetParentEntityId(const AZ::EntityId& frameEntityId) const
    {
        if (m_frameParent.contains(frameEntityId))
        {
            return m_frameParent.find(frameEntityId)->second;
        }
        return AZ::EntityId();
    }

    AZStd::vector<AZ::EntityId> ROS2FrameSystemComponent::FindFrameParentPath(AZ::EntityId frameEntityId)
    {
        AZStd::vector<AZ::EntityId> path;
        path.push_back(frameEntityId);

        AZ::Entity* currentEntity = nullptr;
        AZ::ComponentApplicationBus::BroadcastResult(currentEntity, &AZ::ComponentApplicationRequests::FindEntity, frameEntityId);
        auto* currentTransform = GetEntityTransformInterface(currentEntity);
        if (!currentTransform)
        {
            return path;
        }
        while (currentEntity->GetId().IsValid())
        {
            AZ::EntityId nextEntityId;
            if (!currentTransform)
            {
                return path;
            }
            nextEntityId = currentTransform->GetParentId();
            AZ::Entity* parent = nullptr;

            if (!nextEntityId.IsValid())
            { // Found top of the level
                path.push_back(nextEntityId);
                return path;
            }
            else
            {
                AZ::ComponentApplicationBus::BroadcastResult(parent, &AZ::ComponentApplicationRequests::FindEntity, nextEntityId);
                if (parent == nullptr)
                {
                    path.push_back(nextEntityId);
                    return path;
                }
            }
            if (ROS2FrameComponentBus::HasHandlers(nextEntityId))
            {
                path.push_back(nextEntityId);
                break;
            }
            path.push_back(nextEntityId);
            currentEntity = parent;
            currentTransform = GetEntityTransformInterface(currentEntity);
        }

        return path;
    }

    void ROS2FrameSystemComponent::RegisterFrame(const AZ::EntityId& frameToRegister)
    {
        // Check if the frame is valid and if it's already registered;
        if (!frameToRegister.IsValid() || m_frameParent.contains(frameToRegister))
        {
            return;
        }

        // Get path from itself to its parent (both included)
        AZStd::vector<AZ::EntityId> entityPath = FindFrameParentPath(frameToRegister);
        const AZ::EntityId& frameParent = entityPath[entityPath.size() - 1];

        // Initialize all maps
        AZStd::set<AZ::EntityId>& frameToRegisterChildren = m_frameChildren.insert({ frameToRegister, {} }).first->second;
        AZStd::set<AZ::EntityId>& frameToRegisterWatchedEntities = m_watchedEntities.insert({ frameToRegister, {} }).first->second;
        m_frameParent.insert({ frameToRegister, frameParent });

        bool laysOnPath = false;

        if (auto frameChildrenIt = m_frameChildren.find(frameParent); frameChildrenIt != m_frameChildren.end())
        {
            AZStd::set<AZ::EntityId>& parentsChildren = frameChildrenIt->second;
            // Check if a frameToRegister lays on one or more childrens paths
            AZStd::vector<AZ::EntityId> childrenToErase;
            for (const AZ::EntityId& parentsChild : parentsChildren)
            {
                AZStd::set<AZ::EntityId>& childsPathToParent = m_watchedEntities.find(parentsChild)->second;
                if (childsPathToParent.contains(frameToRegister))
                {
                    laysOnPath = true;
                    // As the new frame lays on the path it is needed to update all entities on the path.
                    for (AZ::EntityId& entityOnPath : entityPath)
                    {
                        if (entityOnPath == frameParent)
                        {
                            // Skip updating its parent
                            continue;
                        }
                        // Update the handler to notify the newly registered frame
                        ROS2FrameSystemTransformHandler& handler = m_watchedEntitiesHandlers.find(entityOnPath)->second;
                        handler.RemoveFrameEntity(parentsChild);
                        handler.AddFrameEntity(frameToRegister);

                        // The child no longer watches the entity
                        childsPathToParent.erase(entityOnPath);
                        frameToRegisterWatchedEntities.insert(entityOnPath);
                    }
                    // Change the parenthood
                    childrenToErase.push_back(parentsChild);
                    frameToRegisterChildren.insert(parentsChild);
                    m_frameParent.find(parentsChild)->second = frameToRegister;
                }
            }
            for (const AZ::EntityId& child : childrenToErase)
            {
                parentsChildren.erase(child);
            }

            // Add itself as a parents child
            parentsChildren.insert(frameToRegister);
        }
        else
        {
            // As its the first frame, add it as a levels child
            m_frameChildren.insert({ frameParent, { frameToRegister } });
            // Add the parent of the level to be the level
            m_frameParent.insert({ frameParent, frameParent });
        }
        if (!laysOnPath)
        {
            // Create new handlers
            for (AZ::EntityId& entityOnPath : entityPath)
            {
                if (entityOnPath == frameParent)
                {
                    // Skip updating its parent
                    continue;
                }
                frameToRegisterWatchedEntities.insert(entityOnPath);
                ROS2FrameSystemTransformHandler& handler =
                    m_watchedEntitiesHandlers.insert({ entityOnPath, ROS2FrameSystemTransformHandler() }).first->second;
                // Add which entity should be notified
                handler.AddFrameEntity(frameToRegister);
                // Connect the handler
                handler.BusConnect(entityOnPath);
            }
        }

        // Update namespaces
        UpdateNamespaces(frameToRegister, frameParent);

        auto predecessors = GetAllPredecessors(frameToRegister);
        for (const auto& predecessor : predecessors)
        {
            ROS2FrameComponentNotificationBus::Event(
                predecessor, &ROS2FrameComponentNotificationBus::Events::OnChildAdded, frameToRegister);
        }
    }

    void ROS2FrameSystemComponent::UnregisterFrame(const AZ::EntityId& frameToUnregister)
    {
        // Check if the frame is already unregistered and valid
        if (!frameToUnregister.IsValid() || !m_frameParent.contains(frameToUnregister))
        {
            return;
        }

        auto predecessors = GetAllPredecessors(frameToUnregister);

        AZStd::set<AZ::EntityId>& frameToUnregisterChildren = m_frameChildren.find(frameToUnregister)->second;
        AZStd::set<AZ::EntityId>& frameToUnregisterWatchedEntities = m_watchedEntities.find(frameToUnregister)->second;
        const AZ::EntityId& frameToUnregisterParent = m_frameParent.find(frameToUnregister)->second;
        AZStd::set<AZ::EntityId>& parentsChildren = m_frameChildren.find(frameToUnregisterParent)->second;

        if (frameToUnregisterChildren.empty())
        {
            for (const AZ::EntityId& watchedEntity : frameToUnregisterWatchedEntities)
            {
                ROS2FrameSystemTransformHandler& handler = m_watchedEntitiesHandlers.find(watchedEntity)->second;
                handler.RemoveFrameEntity(frameToUnregister);
                if (handler.GetFrameCount() == 0)
                {
                    handler.BusDisconnect();
                    m_watchedEntitiesHandlers.erase(watchedEntity);
                }
            }
        }
        else
        {
            for (const AZ::EntityId child : frameToUnregisterChildren)
            {
                // Change the handlers and watched entities
                AZStd::set<AZ::EntityId>& childsWatchedEntities = m_watchedEntities.find(child)->second;
                for (const AZ::EntityId& watchedEntity : frameToUnregisterWatchedEntities)
                {
                    childsWatchedEntities.insert(watchedEntity);
                    ROS2FrameSystemTransformHandler& handler = m_watchedEntitiesHandlers.find(watchedEntity)->second;
                    handler.RemoveFrameEntity(frameToUnregister);
                    handler.AddFrameEntity(child);
                }

                // Change the parenthood
                parentsChildren.insert(child);
                m_frameParent.find(child)->second = frameToUnregisterParent;
            }
        }

        // Update namespaces before removal of children
        UpdateNamespaces(frameToUnregister, frameToUnregisterParent, false);

        // Remove all references to the unregisteredEntity
        m_frameChildren.erase(frameToUnregister);
        m_frameParent.erase(frameToUnregister);
        m_watchedEntities.erase(frameToUnregister);

        // Remove itself form the parents children
        parentsChildren.erase(frameToUnregister);

        // Check if the parents children are empty to ensure that the level parent will be deleted
        if (parentsChildren.size() == 0)
        {
            if (m_frameParent.find(frameToUnregisterParent)->second == frameToUnregisterParent)
            { // Remove the level entity from the registry
                m_frameChildren.erase(frameToUnregisterParent);
                m_frameParent.erase(frameToUnregisterParent);
            }
        }

        for (const auto& predecessor : predecessors)
        {
            ROS2FrameComponentNotificationBus::Event(
                predecessor, &ROS2FrameComponentNotificationBus::Events::OnChildRemoved, frameToUnregister);
        }
    }

    void ROS2FrameSystemComponent::MoveFrameDetach(
        const AZ::EntityId& frameEntityId, const AZStd::set<AZ::EntityId>& newPathToParentFrameSet)
    {
        // Remove all handlers
        AZStd::set<AZ::EntityId>& oldWatchedEntities = m_watchedEntities.find(frameEntityId)->second;
        AZStd::vector<AZ::EntityId> watchedEntitiesToRemove;
        for (const AZ::EntityId& oldWatchedEntity : oldWatchedEntities)
        {
            if (newPathToParentFrameSet.contains(oldWatchedEntity))
            {
                continue;
            }
            ROS2FrameSystemTransformHandler& handler = m_watchedEntitiesHandlers.find(oldWatchedEntity)->second;
            handler.RemoveFrameEntity(frameEntityId);
            watchedEntitiesToRemove.push_back(oldWatchedEntity);
            if (handler.GetFrameCount() == 0)
            {
                handler.BusDisconnect();
                m_watchedEntitiesHandlers.erase(oldWatchedEntity);
            }
        }
        for (const auto& entityIdToRemove : watchedEntitiesToRemove)
        {
            oldWatchedEntities.erase(entityIdToRemove);
        }

        // Remove itself from parents children
        m_frameChildren.find(m_frameParent.find(frameEntityId)->second)->second.erase(frameEntityId);
    }

    void ROS2FrameSystemComponent::MoveFrameAttach(
        const AZ::EntityId& frameEntityId, const AZ::EntityId& newFrameParent, const AZStd::vector<AZ::EntityId>& newPathToParentFrame)
    {
        AZStd::set<AZ::EntityId>& oldWatchedEntities = m_watchedEntities.find(frameEntityId)->second;

        // Add itself as a new child
        m_frameChildren.find(newFrameParent)->second.insert(frameEntityId);

        // Create or add the frame to handlers
        for (const AZ::EntityId& entityIdOnPath : newPathToParentFrame)
        {
            if (entityIdOnPath == newFrameParent || oldWatchedEntities.contains(entityIdOnPath))
            {
                continue;
            }

            if (m_watchedEntitiesHandlers.contains(entityIdOnPath))
            {
                m_watchedEntitiesHandlers.find(entityIdOnPath)->second.AddFrameEntity(frameEntityId);
            }
            else
            {
                ROS2FrameSystemTransformHandler& handler =
                    m_watchedEntitiesHandlers.insert({ entityIdOnPath, ROS2FrameSystemTransformHandler() }).first->second;
                // Add which entity should be notified
                handler.AddFrameEntity(frameEntityId);
                handler.BusConnect(entityIdOnPath);
            }
            oldWatchedEntities.insert(entityIdOnPath);
        }
    }

    void ROS2FrameSystemComponent::MoveFrame(const AZ::EntityId& frameEntityId, const AZ::EntityId& newParent)
    {
        // Check if the frame is already registered and valid
        if (!frameEntityId.IsValid() || !m_frameParent.contains(frameEntityId))
        {
            return;
        }

        AZStd::vector<AZ::EntityId> newPathToParentFrame = FindFrameParentPath(frameEntityId);
        const AZ::EntityId& newFrameParent = newPathToParentFrame[newPathToParentFrame.size() - 1];
        AZStd::set<AZ::EntityId> newPathToParentFrameSet;

        for (const auto& entityOnPath : newPathToParentFrame)
        {
            newPathToParentFrameSet.insert(entityOnPath);
        }

        auto oldPredecessors = GetAllPredecessors(frameEntityId);
        auto successors = GetAllSuccessors(frameEntityId);
        successors.push_back(frameEntityId);

        MoveFrameDetach(frameEntityId, newPathToParentFrameSet);

        for (const auto& successor : successors)
        {
            for (const auto& oldPredecessor : oldPredecessors)
            {
                ROS2FrameComponentNotificationBus::Event(
                    oldPredecessor, &ROS2FrameComponentNotificationBus::Events::OnChildRemoved, successor);
            }
        }

        // Replace the parent
        m_frameParent.find(frameEntityId)->second = newFrameParent;

        MoveFrameAttach(frameEntityId, newFrameParent, newPathToParentFrame);

        auto newPredecessors = GetAllPredecessors(frameEntityId);

        for (const auto& successor : successors)
        {
            for (const auto& newPredecessor : newPredecessors)
            {
                ROS2FrameComponentNotificationBus::Event(
                    newPredecessor, &ROS2FrameComponentNotificationBus::Events::OnChildAdded, successor);
            }
        }

        // Notify about namespace changes
        UpdateNamespaces(frameEntityId, newFrameParent);
    }

    void ROS2FrameSystemComponent::UpdateNamespaces(AZ::EntityId frameEntity, AZ::EntityId frameParentEntity, bool isActive)
    {
        AZStd::string ros2Namespace;
        ROS2FrameComponentBus::EventResult(ros2Namespace, frameParentEntity, &ROS2FrameComponentBus::Events::GetNamespace);
        UpdateNamespaces(frameEntity, ros2Namespace, isActive);
    }

    void ROS2FrameSystemComponent::UpdateNamespaces(AZ::EntityId frameEntity, AZStd::string parentNamespace, bool isActive)
    {
        ROS2FrameComponentBus::Event(frameEntity, &ROS2FrameComponentBus::Events::UpdateNamespace, parentNamespace);
        const AZStd::set<AZ::EntityId>& children = m_frameChildren.find(frameEntity)->second;
        AZStd::string ros2Namespace;
        if (isActive)
        {
            ROS2FrameComponentBus::EventResult(ros2Namespace, frameEntity, &ROS2FrameComponentBus::Events::GetNamespace);
        }
        else
        {
            ros2Namespace = parentNamespace;
        }
        for (const AZ::EntityId& child : children)
        {
            UpdateNamespaces(child, ros2Namespace);
        }
    }

    void ROS2FrameSystemComponent::NotifyChange(const AZ::EntityId& frameEntityId)
    {
        if (frameEntityId.IsValid() && m_frameParent.contains(frameEntityId))
        {
            // Notify about namespace changes
            UpdateNamespaces(frameEntityId, m_frameParent.find(frameEntityId)->second);
        }
    }

    AZStd::set<AZ::EntityId> ROS2FrameSystemComponent::GetChildrenEntityId(const AZ::EntityId& frameEntityId) const
    {
        if (!frameEntityId.IsValid() || !m_frameChildren.contains(frameEntityId))
        {
            return AZStd::set<AZ::EntityId>();
        }

        return m_frameChildren.find(frameEntityId)->second;
    }

    AZStd::vector<AZ::EntityId> ROS2FrameSystemComponent::GetAllPredecessors(const AZ::EntityId& frameEntityId) const
    {
        AZStd::vector<AZ::EntityId> predecessors;
        auto childrenIt = m_frameParent.find(frameEntityId);
        if (childrenIt == m_frameParent.end())
        {
            return predecessors;
        }
        AZ::EntityId currentEntityId = childrenIt->second;
        while (m_frameParent.find(currentEntityId) != m_frameParent.end() && m_frameParent.find(currentEntityId)->second != currentEntityId)
        {
            predecessors.push_back(currentEntityId);
            currentEntityId = m_frameParent.find(currentEntityId)->second;
        }

        return predecessors;
    }

    AZStd::vector<AZ::EntityId> ROS2FrameSystemComponent::GetAllSuccessors(const AZ::EntityId& frameEntityId) const
    {
        AZStd::vector<AZ::EntityId> successors;
        AZStd::set<AZ::EntityId> children = GetChildrenEntityId(frameEntityId);
        for (const AZ::EntityId& child : children)
        {
            successors.push_back(child);
            AZStd::vector<AZ::EntityId> childSuccessors = GetAllSuccessors(child);
            successors.insert(successors.end(), childSuccessors.begin(), childSuccessors.end());
        }

        return successors;
    }

} // namespace ROS2
