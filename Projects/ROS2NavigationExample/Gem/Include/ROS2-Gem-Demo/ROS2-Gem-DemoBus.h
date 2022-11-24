/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/EBus/EBus.h>
#include <AzCore/Interface/Interface.h>

namespace RobotVacuumSample
{
    class RobotVacuumSampleRequests
    {
    public:
        AZ_RTTI(RobotVacuumSampleRequests, "{16673c77-215c-41c6-8bbf-6cd2412568e9}");
        virtual ~RobotVacuumSampleRequests() = default;
    };

    class RobotVacuumSampleBusTraits
        : public AZ::EBusTraits
    {
    public:
        //////////////////////////////////////////////////////////////////////////
        // EBusTraits overrides
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::Single;
        //////////////////////////////////////////////////////////////////////////
    };

    using RobotVacuumSampleRequestBus = AZ::EBus<RobotVacuumSampleRequests, RobotVacuumSampleBusTraits>;
    using RobotVacuumSampleInterface = AZ::Interface<RobotVacuumSampleRequests>;

} // namespace RobotVacuumSample
