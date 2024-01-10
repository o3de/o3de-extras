/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/Component/EntityId.h>
#include <AzCore/EBus/EBusSharedDispatchTraits.h>
#include <AzCore/Interface/Interface.h>
#include <AzCore/std/string/string.h>
#include <sensor_msgs/msg/image.hpp>

namespace ROS2
{
    //! Interface class that allows to add post-processing to the pipeline
    //!
    //! Each function call can be processed without blocking Bus for other dispatches.
    //! Do not use connects / disconnects to this bus during event dispatch, as they are not allowed for this concurrency model.
    //! Those constraints allow for processing multiple camera frames in the same time.
    //! Bus implementations should allow for asynchronous execution of provided functions.
    class CameraPostProcessingRequests : public AZ::EBusSharedDispatchTraits<CameraPostProcessingRequests>
    {
    public:
        //! Each camera sensor component has its own post-processing bus.
        using BusIdType = AZ::EntityId;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::ById;
        //! Multiple post-processing functions can be registered to the bus.
        //! They will be executed in the order
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::MultipleAndOrdered;

        //! Priority of the post-processing bus.
        //! @note higher priority buses will be processed first.
        static constexpr AZ::u8 MIN_PRIORITY = 0;
        static constexpr AZ::u8 MAX_PRIORITY = 255;
        static constexpr AZ::u8 DEFAULT_PRIORITY = 127;

        //! Apply post-processing function, if any implementations to the bus are in the entity.
        //! @param image standard image message passed as a reference. It will be changed through post-processing.
        //! @note Handler should always handle function call,
        //! if handler does not support the format of the image, it should not change the image.
        //! @note Every image is modified in place. Only one handler has access to the image at the time.
        virtual void ApplyPostProcessing(sensor_msgs::msg::Image& image) = 0;

        //! Get priority of the post-processing bus.
        //! @return priority of the bus.
        //! @note higher priority buses will be processed first.
        virtual AZ::u8 GetPriority() const = 0;

        //! Compare function required by BusHandlerOrderCompare = BusHandlerCompareDefault
        //! \param[in] other Another instance of the class to compare
        //! @return True if this bus should be processed before the other.
        inline bool Compare(const CameraPostProcessingRequests* other) const
        {
            return GetPriority() > other->GetPriority();
        }
    };

    using CameraPostProcessingRequestBus = AZ::EBus<CameraPostProcessingRequests>;
} // namespace ROS2
