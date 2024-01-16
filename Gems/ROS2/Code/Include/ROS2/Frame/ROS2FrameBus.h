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
#include <ROS2/Frame/ROS2Transform.h>
#include <ROS2/ROS2GemUtilities.h>

namespace ROS2
{
    //! Interface for the ROS2FrameEditorComponent.
    //! Used to change and get information about the ROS2FrameEditorComponent.
    class ROS2FrameComponentRequests : public AZ::ComponentBus
    {
    public:
        // One handler per address is supported.
        static const AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;

        // The EBus has multiple addresses. Addresses are not ordered.
        static const AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::ById;

        // Messages are addressed by EntityId.
        using BusIdType = AZ::EntityId;

        //! Get a frame id, which is needed for any ROS2 message with a Header
        //! @return Frame id which includes the namespace, ready to send in a ROS2 message
        virtual AZStd::string GetFrameID() const = 0;

        //! Get the joint name including the namespace
        //! @note Supplementary metadata for Joint components, necessary in some cases for joints addressed by name in ROS 2
        //! @return The namespaced joint name, ready to send in a ROS2 message
        virtual AZ::Name GetJointName() const = 0;

        //! Get a namespace, which should be used for any publisher or subscriber in the same entity.
        //! @return A complete namespace (including parent namespaces)
        virtual AZStd::string GetNamespace() const = 0;

        //! Find the parent frame of the entity.
        //! @return entityId of the parent frame or an invalid entityId if the frame is top level.
        virtual AZ::EntityId GetFrameParent() const = 0;

        //! Find all frame children of the frame.
        //! @return set of all entityIds of children. Empty if no children or the frameEntityId is invalid.
        virtual AZStd::set<AZ::EntityId> GetFrameChildren() const = 0;

        //! Update the parent namespace and effective namespace.
        virtual void UpdateNamespace(const AZStd::string& parentNamespace) = 0;

        //! Global frame name in ros2 ecosystem.
        //! @return The name of the global frame with namespace attached. It is typically "odom", "map", "world".
        virtual AZStd::string GetGlobalFrameName() const = 0;

        //! Check if the ROS 2 frame is top level.
        //! @return true if the ROS 2 Frame component has no frame parent.
        virtual bool IsTopLevel() const = 0;
    };

    using ROS2FrameComponentBus = AZ::EBus<ROS2FrameComponentRequests>;

    class ROS2FrameComponentNotifications : public AZ::ComponentBus
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

    using ROS2FrameComponentNotificationBus = AZ::EBus<ROS2FrameComponentNotifications>;
} // namespace ROS2
