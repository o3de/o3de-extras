// {BEGIN_LICENSE}
/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
// {END_LICENSE}

#pragma once

#include <AzCore/EBus/EBus.h>
#include <AzCore/Interface/Interface.h>

namespace ROS2_Gem_Demo
{
    class ROS2_Gem_DemoRequests
    {
    public:
        AZ_RTTI(ROS2_Gem_DemoRequests, "{0347d2ff-c2db-471f-8ad2-0408d808cc91}");
        virtual ~ROS2_Gem_DemoRequests() = default;
        // Put your public methods here
    };

    class ROS2_Gem_DemoBusTraits
        : public AZ::EBusTraits
    {
    public:
        //////////////////////////////////////////////////////////////////////////
        // EBusTraits overrides
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::Single;
        //////////////////////////////////////////////////////////////////////////
    };

    using ROS2_Gem_DemoRequestBus = AZ::EBus<ROS2_Gem_DemoRequests, ROS2_Gem_DemoBusTraits>;
    using ROS2_Gem_DemoInterface = AZ::Interface<ROS2_Gem_DemoRequests>;

} // namespace ROS2_Gem_Demo
