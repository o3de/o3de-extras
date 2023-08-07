/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzFramework/Components/TransformComponent.h>
#include <ROS2/Frame/ROS2Transform.h>
#include <ROS2/ROS2GemUtilities.h>

namespace ROS2
{
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

        //! Get a transform between this frame and the next frame up in hierarchy.
        //! @return If the parent frame is found, return a Transform between this frame and the parent.
        //! Otherwise, return a global Transform.
        //! @note Parent frame is not the same as parent Transform: there could be many Transforms in between without ROS2Frame components.
        virtual AZ::Transform GetFrameTransform() const = 0;

        //! Global frame name in ros2 ecosystem.
        //! @return The name of the global frame with namespace attached. It is typically "odom", "map", "world".
        virtual AZStd::string GetGlobalFrameName() const = 0;

        virtual bool IsFrame() const = 0;
    };

    using ROS2FrameComponentBus = AZ::EBus<ROS2FrameComponentRequests>;
} // namespace ROS2
