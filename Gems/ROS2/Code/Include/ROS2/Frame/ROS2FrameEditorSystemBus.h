/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Component/ComponentBus.h>
#include <AzCore/Component/EntityId.h>
#include <AzCore/EBus/EBus.h>
#include <AzCore/EBus/Policies.h>
#include <AzCore/Interface/Interface.h>
#include <AzCore/RTTI/RTTIMacros.h>
#include <AzCore/std/containers/set.h>
#include <AzCore/std/string/string.h>

namespace ROS2
{
    class ROS2FrameEditorSystemRequests
    {
    public:
        AZ_RTTI(ROS2FrameEditorSystemRequests, "{24FE4584-0499-4A37-BC1A-00CA04BD22F5}");

        //! Move the frame in the entity tree.
        //! Moves the frame entity and updates all namespaces.
        //! Used by the ROS2FrameSystemComponent to change the frames configuration after entity move in the editor.
        //! @param frameEntityId entityId of the frame to move.
        //! @param newParent entityId of the new parent of the moved frame (does not need to be a entity
        //! containing a frame component).
        virtual void MoveFrame(const AZ::EntityId& frameEntityId, const AZ::EntityId& newParent) = 0;

        //! Notify the system entity about frames configuration change.
        //! This function should be called when a frame entity has changed its reflected configuration.
        //! @param frameEntityId entityId of the frame components entity that has changed its configuration.
        virtual void NotifyChange(const AZ::EntityId& frameEntityId) = 0;

        //! Check if the frame is the highest frame in the entity tree.
        //! @param frameEntityId entityId of the frame to check.
        //! @return boolean value of the check. True for top level.
        virtual bool IsTopLevel(const AZ::EntityId& frameEntityId) const = 0;

        //! Find the parent frame of the entity.
        //! @param frameEntityId entityId of the frame to check.
        //! @return entityId of the parent frame or an invalid entityId if the frame is top level.
        virtual AZ::EntityId GetParentEntityId(const AZ::EntityId& frameEntityId) const = 0;

        //! Find all frame children of the frame.
        //! @param frameEntityId entityId of the frame to check.
        //! @return set of all entityIds of children. Empty if no children or the frameEntityId is invalid.
        virtual AZStd::set<AZ::EntityId> GetChildrenEntityId(const AZ::EntityId& frameEntityId) const = 0;
    };

    class ROS2FrameEditorSystemBusTraits : public AZ::EBusTraits
    {
    public:
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::Single;
    };

    using ROS2FrameEditorSystemInterface = AZ::Interface<ROS2FrameEditorSystemRequests>;

    // An internal bus used by the ROS2FrameEditorComponent to update the namespace of the component. This Bus is only used
    // by the ROS2FrameEditorComponent and EditorSystemComponent.
    class ROS2FrameInternalComponentRequests : public AZ::ComponentBus
    {
    public:
        AZ_RTTI(ROS2FrameInternalComponentRequests, "{52221A90-9DBD-4834-B661-C080D188B4B3}");

        static const AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static const AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::ById;
        using BusIdType = AZ::EntityId;

        //! Update the parent namespace and effective namespace.
        //! This method should be called when updating the namespaces of all children of the frameEntity with changed namespace.
        //! @param parentNamespace The namespace of the parent frame.
        virtual void UpdateNamespace(const AZStd::string& parentNamespace) = 0;
    };

    using ROS2FrameInternalComponentBus = AZ::EBus<ROS2FrameInternalComponentRequests>;
} // namespace ROS2
