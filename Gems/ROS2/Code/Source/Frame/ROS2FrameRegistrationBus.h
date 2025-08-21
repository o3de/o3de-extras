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

namespace ROS2
{
    //! Bus for registering and managing ROS2 frame components.
    //! This interface handles the basic lifecycle management of ROS2 frame components.
    class ROS2FrameRegistrationRequests
    {
    public:
        AZ_RTTI(ROS2FrameRegistrationRequests, "{A1B2C3D4-E5F6-7890-ABCD-EF1234567890}");

        //! Register a ROS2FrameComponent with the system.
        //! All ROS2FrameComponents should register using this function during activation.
        //! @param frameEntityId EntityId containing the frame to register
        virtual void RegisterFrame(const AZ::EntityId& frameEntityId) = 0;

        //! Unregister a ROS2FrameComponent from the system.
        //! All ROS2FrameComponents should unregister using this function during deactivation.
        //! @param frameEntityId EntityId containing the frame to unregister
        virtual void UnregisterFrame(const AZ::EntityId& frameEntityId) = 0;
    };

    class ROS2FrameRegistrationBusTraits : public AZ::EBusTraits
    {
    public:
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::Single;
    };

    using ROS2FrameRegistrationInterface = AZ::Interface<ROS2FrameRegistrationRequests>;
    using ROS2FrameRegistrationBus = AZ::EBus<ROS2FrameRegistrationRequests, ROS2FrameRegistrationBusTraits>;
} // namespace ROS2
