/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/Component/ComponentBus.h>
#include <ROS2/ROS2TypeIds.h>

namespace ROS2
{
    class ROS2FrameComponentRequests : public AZ::ComponentBus
    {
    public:
        AZ_RTTI(ROS2FrameComponentRequests, ROS2FrameComponentRequestsTypeId);
        // One handler per address is supported.
        static const AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;

        // The EBus has multiple addresses. Addresses are not ordered.
        static const AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::ById;

        // Messages are addressed by EntityId.
        using BusIdType = AZ::EntityId;

        //! Get a namespace, which should be used for any publisher or subscriber in the same entity.
        //! @return A complete namespace (including parent namespaces)
        virtual AZStd::string GetNamespace() const = 0;

        //! Get a frame id, which is needed for any ROS2 message with a Header
        //! @return Frame id which includes the namespace, ready to send in a ROS2 message
        virtual AZStd::string GetNamespacedFrameID() const = 0;

        //! Get the joint name including the namespace
        //! @note Supplementary metadata for Joint components, necessary in some cases for joints addressed by name in ROS 2
        //! @return The namespaced joint name, ready to send in a ROS2 message
        virtual AZStd::string GetNamespacedJointName() const = 0;

        //! Get the joint name without namespace
        //! @return The joint name without namespace prefix
        virtual AZStd::string GetJointName() const = 0;

        //! Get the frame name without namespace
        //! @return The frame name without namespace prefix
        virtual AZStd::string GetFrameName() const = 0;

        //! Global frame name in ros2 ecosystem with namespace.
        //! It is configurable through Settings Registry at /O3DE/ROS2/GlobalFrameName
        //! If not configured, it defaults to "odom".
        //! If empty, root frame is not published by the simulator.
        //! It is typically "odom", "map", "world", "myRobot/odom"
        virtual AZStd::string GetGlobalFrameID() const = 0;
    };

    using ROS2FrameComponentBus = AZ::EBus<ROS2FrameComponentRequests>;
} // namespace ROS2
