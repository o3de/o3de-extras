/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Component/Component.h>
#include <AzCore/Component/Entity.h>
#include <AzCore/Component/EntityId.h>
#include <AzCore/Component/TransformBus.h>
#include <AzCore/Math/Transform.h>
#include <AzCore/RTTI/RTTIMacros.h>
#include <AzCore/std/containers/map.h>
#include <AzCore/std/containers/vector.h>
#include <AzCore/std/parallel/mutex.h>
#include <AzCore/std/string/string.h>
#include <ROS2/Frame/ROS2FrameSystemBus.h>

namespace ROS2
{
    // Handler class for the ROS2FrameEditorComponent. It watches for changes in the entity tree and notifies about moves.
    class ROS2FrameSystemTransformHandler : public AZ::TransformNotificationBus::Handler
    {
    public:
        AZ_RTTI(ROS2FrameSystemTransformHandler, "{0b4324bb-4c36-4a86-8827-a044a6ac44e8}");

        //! Override of the AZ::TransformNotificationBus.
        void OnParentChanged(AZ::EntityId oldParent, AZ::EntityId newParent) override;

        //! Add a frame entity which should be notified about a change in the tree.
        //! @param frameEntityId frame to notify.
        void AddFrameEntity(AZ::EntityId frameEntityId);

        //! Remove a frame entity which shouldn't be notified about a change in the tree.
        //! @param frameEntityId frame to remove.
        void RemoveFrameEntity(AZ::EntityId frameEntityId);

        //! Get size of the m_frameEntities.
        //! @return size of the m_frameEntities.
        unsigned int GetFrameCount();

    private:
        AZStd::set<AZ::EntityId> m_frameEntities;
    };

    class ROS2FrameSystemComponent
        : public AZ::Component
        , public ROS2FrameSystemInterface::Registrar
    {
    public:
        AZ_COMPONENT(ROS2FrameSystemComponent, "{360c4b45-ac02-42d2-9e1a-1d77eb22a054}");
        static void Reflect(AZ::ReflectContext* context);

        // AZ::Component overrides.
        void Activate() override;
        void Deactivate() override;

        // ROS2FrameSystemInterface::Registrar overrides.
        void RegisterFrame(const AZ::EntityId& frameEntityId) override;
        void UnregisterFrame(const AZ::EntityId& frameEntityId) override;
        void MoveFrame(const AZ::EntityId& frameEntityId, const AZ::EntityId& newParent) override;
        void NotifyChange(const AZ::EntityId& frameEntityId) override;
        bool IsTopLevel(const AZ::EntityId& frameEntityId) const override;
        AZ::EntityId GetParentEntityId(const AZ::EntityId& frameEntityId) const override;
        AZStd::set<AZ::EntityId> GetChildrenEntityId(const AZ::EntityId& frameEntityId) const override;

        ROS2FrameSystemComponent();

        ~ROS2FrameSystemComponent();

    private:
        //! Find the path from the frameEntity to the frame parent of that entity.
        //! This path will include the frameEntity and the frame parent.
        //! If there is no frame parent, path to the root entity (included) will be returned.
        //! @param frameEntityId frame to find the path to the parent.
        //! @return vector of entityIds which represent the path to the parent. frameEntityId is first, parent is last.
        AZStd::vector<AZ::EntityId> FindFrameParentPath(AZ::EntityId frameEntityId);

        AZ::TransformInterface* GetEntityTransformInterface(const AZ::Entity* entity);

        //! Updates the namespaces of all children of the frameEntity.
        //! @param frameEntity frame to be updated.
        //! @param parentNamespace namespace of the parent frame. Empty if no parent is present.
        //! @param isActive boolean value describing if the frameEntity is currently active.
        void UpdateNamespaces(AZ::EntityId frameEntity, AZStd::string parentNamespace = "", bool isActive = true);

        //! Updates the namespaces of all children of the frameEntity.
        //! @param frameEntity frame to be updated.
        //! @param frameParentEntity entityId of the parent frame.
        //! @param isActive boolean value describing if the frameEntity is currently active.
        void UpdateNamespaces(AZ::EntityId frameEntity, AZ::EntityId frameParentEntity, bool isActive = true);

        void MoveFrameDetach(const AZ::EntityId& frameEntityId, const AZStd::set<AZ::EntityId>& newPathToParentFrameSet);
        void MoveFrameAttach(
            const AZ::EntityId& frameEntityId, const AZ::EntityId& newFrameParent, const AZStd::vector<AZ::EntityId>& newPathToParentFrame);

        AZStd::map<AZ::EntityId, AZStd::set<AZ::EntityId>> m_frameChildren;
        AZStd::map<AZ::EntityId, AZ::EntityId> m_frameParent;
        AZStd::map<AZ::EntityId, AZStd::set<AZ::EntityId>> m_watchedEntities;
        AZStd::map<AZ::EntityId, ROS2FrameSystemTransformHandler> m_watchedEntitiesHandlers;
    };
} // namespace ROS2
