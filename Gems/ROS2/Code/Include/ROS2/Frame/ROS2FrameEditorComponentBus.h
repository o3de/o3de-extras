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
#include <AzCore/std/containers/set.h>
#include <AzFramework/Components/TransformComponent.h>

namespace ROS2
{
    //! Interface for the ROS2FrameEditorComponent.
    //! Used to change and get information about the ROS2FrameEditorComponent.
    class ROS2FrameEditorComponentRequests : public AZ::ComponentBus
    {
    public:
        // One handler per address is supported.
        static const AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;

        // The EBus has multiple addresses. Addresses are not ordered.
        static const AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::ById;

        // Messages are addressed by EntityId.
        using BusIdType = AZ::EntityId;

        //! Find the parent frame of the entity.
        //! @return entityId of the parent frame or an invalid entityId if the frame is top level.
        virtual AZ::EntityId GetFrameParent() const = 0;

        //! Find all frame descendants of the frame.
        //! @return set of all entityIds of children. Empty if no children or the frameEntityId is invalid.
        virtual AZStd::set<AZ::EntityId> GetFrameDescendants() const = 0;

        //! Ask component to recompute its namespace based on the hierarchy and its configuration.
        //! called when some changes happen in the o3de transform tree or when configuration changes.
        virtual void UpdateNamespace() = 0;

        //! Set the joint name (excluding namespace).
        //! @note May be populated during robot import.
        //! @param jointName The joint name to set.
        virtual void SetJointName(const AZStd::string& jointName) = 0;

    };

    using ROS2FrameEditorComponentBus = AZ::EBus<ROS2FrameEditorComponentRequests>;

    class ROS2FrameEditorComponentNotifications : public AZ::ComponentBus
    {
    public:
        static const AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Multiple;
        static const AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::ById;

        using BusIdType = AZ::EntityId;

        //! Notification when the ROS2FrameComponent changes its configuration or namespace
        virtual void OnConfigurationChange() = 0;

        //! Notification when the ROS2FrameEditorComponent is added as a child to another ROS2FrameEditorComponent
        virtual void OnChildAdded(AZ::EntityId addedChild) = 0;

        //! Notification when the ROS2FrameEditorComponent is removed as a child from another ROS2FrameEditorComponent
        virtual void OnChildRemoved(AZ::EntityId removedChild) = 0;
    };

    using ROS2FrameEditorComponentNotificationBus = AZ::EBus<ROS2FrameEditorComponentNotifications>;
} // namespace ROS2
