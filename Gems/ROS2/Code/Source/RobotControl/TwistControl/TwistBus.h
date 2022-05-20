/*
* Copyright (c) Contributors to the Open 3D Engine Project.
* For complete copyright and license terms please see the LICENSE at the root of this distribution.
*
* SPDX-License-Identifier: Apache-2.0 OR MIT
*
*/
#pragma once
#include <AzCore/EBus/EBus.h>
#include <AzCore/RTTI/BehaviorContext.h>
#include <AzCore/Math/Vector3.h>

namespace ROS2
{
    class TwistNotifications
        : public AZ::EBusTraits
    {
    public:
        static const AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Multiple;
        static const AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::Single;
        virtual void TwistReceived(const AZ::Vector3& /*linear*/, const AZ::Vector3& /*angular*/) = 0;
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