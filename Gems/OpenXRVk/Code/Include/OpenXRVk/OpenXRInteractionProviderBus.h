/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/EBus/EBus.h>
#include <AzCore/std/string/string.h>
#include <AzCore/std/containers/vector.h>

namespace OpenXRVk
{
    class OpenXRInteractionProvider
        : public AZ::EBusTraits
    {
    public:
        static const AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static const AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::ById;
        typedef AZStd::string BusId;

        virtual AZStd::string GetName() const = 0;
        virtual AZStd::vector<AZStd::string> GetUserPaths() const = 0;
        virtual AZStd::vector<AZStd::string> GetComponentPaths(const AZStd::string& userPath) const = 0;
    };

    using OpenXRInteractionProviderBus = AZ::EBus<OpenXRInteractionProvider>;
}
