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
#include <AzCore/Interface/Interface.h>
#include <AzCore/EBus/EBus.h>

namespace MachineLearning
{
    using ModelSet = AZStd::set<INeuralNetworkPtr>;

    class IMachineLearning
    {
    public:

        AZ_RTTI(IMachineLearning, IMachineLearningTypeId);

        virtual ~IMachineLearning() = default;

        //! Registers a model with the machine learning interface.
        virtual void RegisterModel(INeuralNetworkPtr model) = 0;

        //! Removes a model from the machine learning interface.
        virtual void UnregisterModel(INeuralNetworkPtr model) = 0;

        //! Retrieves the full set of registered models from the machine learning interface.
        virtual ModelSet& GetModelSet() = 0;
    };

    class IMachineLearningBusTraits
        : public AZ::EBusTraits
    {
    public:
        //! EBusTraits overrides
        //! @{
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::Single;
        //! @}
    };

    using MachineLearningRequestBus = AZ::EBus<IMachineLearning, IMachineLearningBusTraits>;
    using MachineLearningInterface = AZ::Interface<IMachineLearning>;
}
