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
#include <ROS2Sensors/Lidar/LidarRaycasterBus.h>
#include <ROS2Sensors/ROS2SensorsTypeIds.h>

namespace ROS2Sensors
{
    //! Interface class that allows for communication with a given Lidar System (implementation).
    class LidarSystemRequests
    {
    public:
        AZ_RTTI(LidarSystemRequests, LidarSystemBusTypeId);

        //! Creates a new Lidar.
        //! @param lidarEntityId EntityId holding the ROS2LidarSensorComponent.
        //! @return A unique Id of the newly created Lidar.
        virtual LidarId CreateLidar(AZ::EntityId lidarEntityId) = 0;

        //! Destroys a no longer used Lidar.
        //! @param lidarId Id of the lidar to be destroyed.
        virtual void DestroyLidar(LidarId lidarId) = 0;

    protected:
        ~LidarSystemRequests() = default;
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
} // namespace ROS2Sensors
