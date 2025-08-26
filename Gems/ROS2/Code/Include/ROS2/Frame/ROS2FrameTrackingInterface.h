/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Component/EntityId.h>
#include <AzCore/EBus/EBus.h>
#include <AzCore/Interface/Interface.h>
#include <AzCore/RTTI/RTTIMacros.h>
#include <AzCore/std/containers/unordered_set.h>
#include <AzCore/std/string/string.h>

namespace ROS2
{
    //! Interface for querying and tracking ROS2 frame components in the system.
    //! This interface provides access to the frame registry maintained by the ROS2FrameSystemComponent.
    class ROS2FrameTrackingRequests
    {
    public:
        AZ_RTTI(ROS2FrameTrackingRequests, "{6B262459-BA1B-4C6C-A9FF-C21D2841CCC1}");

        //! Get all currently registered frame entities.
        //! @return Set of EntityIds representing all registered frames
        virtual const AZStd::unordered_set<AZ::EntityId>& GetRegisteredFrames() const = 0;

        //! Check if a specific frame entity is registered.
        //! @param frameEntityId The EntityId to check
        //! @return True if the frame is registered, false otherwise
        virtual bool IsFrameRegistered(const AZ::EntityId& frameEntityId) const = 0;

        //! Get the number of registered frames.
        //! @return Number of registered frame entities
        virtual size_t GetRegisteredFrameCount() const = 0;

        //! Get the entity ID for a frame with the given namespaced frame ID.
        //! @param namespacedFrameId The namespaced frame ID to search for
        //! @return EntityId of the frame, or invalid EntityId if not found
        virtual AZ::EntityId GetFrameEntityByNamespacedId(const AZStd::string& namespacedFrameId) const = 0;

        //! Get the namespaced frame ID for a given entity.
        //! @param frameEntityId The EntityId to get the namespaced frame ID for
        //! @return The namespaced frame ID, or empty string if not found
        virtual AZStd::string GetNamespacedFrameId(const AZ::EntityId& frameEntityId) const = 0;

        //! Get all namespaced frame IDs currently tracked.
        //! @return Set of all namespaced frame IDs
        virtual AZStd::unordered_set<AZStd::string> GetAllNamespacedFrameIds() const = 0;
    };

    using ROS2FrameTrackingInterface = AZ::Interface<ROS2FrameTrackingRequests>;
} // namespace ROS2
