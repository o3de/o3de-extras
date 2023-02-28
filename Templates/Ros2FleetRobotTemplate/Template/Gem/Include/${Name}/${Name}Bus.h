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

namespace ${Name}
{
    class ${Name}Requests
    {
    public:
        AZ_RTTI(${Name}Requests, "{${Random_Uuid}}");
        virtual ~${Name}Requests() = default;
        // Put your public methods here
    };

    class ${Name}BusTraits
        : public AZ::EBusTraits
    {
    public:
        //////////////////////////////////////////////////////////////////////////
        // EBusTraits overrides
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::Single;
        //////////////////////////////////////////////////////////////////////////
    };

    using ${Name}RequestBus = AZ::EBus<${Name}Requests, ${Name}BusTraits>;
    using ${Name}Interface = AZ::Interface<${Name}Requests>;

} // namespace ${Name}
