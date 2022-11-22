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

namespace ROS2
{
    class LidarSystemRequests
    {
    public:
        AZ_RTTI(LidarSystemRequests, "{007871d1-2783-4382-977b-558f436c54a5}");
        virtual ~LidarSystemRequests() = default;

        //! Creates a new Lidar.
        //! @param lidarEntityId EntityId holding the ROS2LidarSensorComponent.
        //! @return A unique Id of the newly created Lidar.
        virtual AZ::Uuid CreateLidar(const AZ::EntityId& lidarEntityId) = 0;
    };

    class LidarSystemBusTraits : public AZ::EBusTraits
    {
    public:
        //////////////////////////////////////////////////////////////////////////
        // EBusTraits overrides
        using BusIdType = AZ::Crc32;
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Multiple;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::ById;
        //////////////////////////////////////////////////////////////////////////
    };

    using LidarSystemRequestBus = AZ::EBus<LidarSystemRequests, LidarSystemBusTraits>;
} // namespace ROS2