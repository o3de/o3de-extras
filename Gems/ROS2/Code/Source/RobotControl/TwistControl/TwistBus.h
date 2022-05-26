/*
* Copyright (c) Contributors to the Open 3D Engine Project.
* For complete copyright and license terms please see the LICENSE at the root of this distribution.
*
* SPDX-License-Identifier: Apache-2.0 OR MIT
*
*/
#pragma once

#include <AzCore/EBus/EBus.h>
#include <AzCore/Math/Vector3.h>
#include <AzCore/RTTI/BehaviorContext.h>

namespace ROS2
{
    //! Interface class for handling Twist commands through EBus notifications.
    //! The interface serves to enable control through Twist (and TwistStamped) messages.
    class TwistNotifications
        : public AZ::EBusTraits
    {
    public:
        static const AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Multiple;
        static const AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::Single;

        //! Handle control command
        //! @param linear Linear speed in each axis, in robot reference frame, in m/s.
        //! @param angular Angular speed in each axis, in robot reference frame, in m/s.
        virtual void TwistReceived(const AZ::Vector3& linear, const AZ::Vector3& angular) = 0;
    };

    using TwistNotificationBus = AZ::EBus <TwistNotifications>;

    class TwistNotificationHandler
        : public TwistNotificationBus::Handler
        , public AZ::BehaviorEBusHandler
    {
    public:
        AZ_EBUS_BEHAVIOR_BINDER(TwistNotificationHandler, "{CD26E702-6F40-4FF9-816D-4DCB652D97DF}", AZ::SystemAllocator,
            TwistReceived);

        void TwistReceived(const AZ::Vector3& v, const AZ::Vector3 &a) override;
        static void Reflect(AZ::ReflectContext* context);
    };
}