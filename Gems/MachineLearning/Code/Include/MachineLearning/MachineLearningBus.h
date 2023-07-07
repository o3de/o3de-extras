/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <MachineLearning/MachineLearningTypeIds.h>
#include <MachineLearning/INeuralNetwork.h>

#include <AzCore/EBus/EBus.h>
#include <AzCore/Interface/Interface.h>

namespace MachineLearning
{
    class MachineLearningRequests
    {
    public:
        AZ_RTTI(MachineLearningRequests, MachineLearningRequestsTypeId);
        virtual ~MachineLearningRequests() = default;
        // Put your public methods here
    };

    class MachineLearningBusTraits
        : public AZ::EBusTraits
    {
    public:
        //////////////////////////////////////////////////////////////////////////
        // EBusTraits overrides
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::Single;
        //////////////////////////////////////////////////////////////////////////
    };

    using MachineLearningRequestBus = AZ::EBus<MachineLearningRequests, MachineLearningBusTraits>;
    using MachineLearningInterface = AZ::Interface<MachineLearningRequests>;

} // namespace MachineLearning
