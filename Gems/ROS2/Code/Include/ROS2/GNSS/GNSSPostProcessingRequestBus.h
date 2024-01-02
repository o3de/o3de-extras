/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include "AzCore/EBus/EBusSharedDispatchTraits.h"
#include <AzCore/Component/EntityId.h>
#include <AzCore/Interface/Interface.h>
#include <AzCore/std/string/string.h>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

namespace ROS2
{
    //! Interface class that allows to add post-processing to the pipeline
    //!
    //! Each function call can be processed without blocking Bus for other dispatches.
    //! Do not use connects / disconnects to this bus during event dispatch, as they are not allowed for this concurrency model.
    //! Those constraints allow for processing multiple GNSS frames at the same time.
    //! Bus implementations should allow for asynchronous execution of provided functions.
    class GNSSPostProcessingRequests : public AZ::EBusSharedDispatchTraits<GNSSPostProcessingRequests>
    {
    public:
        using BusIdType = AZ::EntityId;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::ById;
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Multiple;

        //! Apply post-processing function, if any implementations to the bus are in the entity.
        //! @param gnss standard GNSS message passed as a reference. It will be changed through post-processing.
        virtual void ApplyPostProcessing(sensor_msgs::msg::NavSatFix& gnss) = 0;

    protected:
        ~GNSSPostProcessingRequests() = default;
    };

    using GNSSPostProcessingRequestBus = AZ::EBus<GNSSPostProcessingRequests>;
} // namespace ROS2
