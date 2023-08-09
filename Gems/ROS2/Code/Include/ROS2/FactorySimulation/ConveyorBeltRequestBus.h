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

namespace ROS2
{
    //! Interface class that allows to add post-processing to the pipeline
    class ConveyorBeltRequests : public AZ::EBusTraits
    {
    public:
        using BusIdType = AZ::EntityId;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::ById;
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;

        //! Start a particular belt.
        virtual void StartBelt() = 0;

        //! Stop a particular belt.
        virtual void StopBelt() = 0;

        //! Query whether a particular conveyor belt is stopped.
        virtual bool IsBeltStopped() = 0;

    protected:
        ~ConveyorBeltRequests() = default;
    };

    using ConveyorBeltRequestBus = AZ::EBus<ConveyorBeltRequests>;
} // namespace ROS2
