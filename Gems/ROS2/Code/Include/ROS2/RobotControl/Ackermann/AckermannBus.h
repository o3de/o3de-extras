/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include "AckermannCommandStruct.h"
#include <AzCore/EBus/EBus.h>
#include <AzCore/RTTI/BehaviorContext.h>

namespace ROS2
{
    //! Interface class for handling Ackermann kinematics steering commands through EBus notifications.
    //! The interface serves to enable control through AckermannDrive (and AckermannDriveStamped) messages.
    class AckermannNotifications : public AZ::EBusTraits
    {
    public:
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::ById;
        using BusIdType = AZ::EntityId;

        //! Handle Ackermann command
        //! @param ackermannCommand A structure with AckermannDrive message fields
        virtual void AckermannReceived(const AckermannCommandStruct& ackermannCommand) = 0;
    };

    using AckermannNotificationBus = AZ::EBus<AckermannNotifications>;
} // namespace ROS2