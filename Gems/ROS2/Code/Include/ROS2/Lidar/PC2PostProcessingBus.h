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
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace ROS2
{
    namespace PC2PostProcessing
    {
        //! Priority of the post-processing bus.
        static constexpr AZ::u8 MinPriority = 0U;
        static constexpr AZ::u8 MaxPriority = 255U;
        static constexpr AZ::u8 DefaultPriority = 127U;
    } // namespace PC2PostProcessing

    //! Interface class that allows for multiple post-processing
    //! operations to be applied on a Point Cloud 2 message.
    //! Post-processing operations defined by handlers of this bus are
    //! executed in descending order based on their respective priorities
    class PC2PostProcessingRequests : public AZ::EBusTraits
    {
    public:
        virtual ~PC2PostProcessingRequests() = default;
        using BusIdType = AZ::EntityId;
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::MultipleAndOrdered;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::ById;

        //! Apply post-processing on a Point Cloud 2 message.
        //! @param message Message to apply post-processing on.
        virtual void ApplyPostProcessing(sensor_msgs::msg::PointCloud2& message) = 0;

        //! Get priority of the post-processing bus.
        //! @return priority of the bus.
        //! @note higher priority buses will be processed first.
        virtual AZ::u8 GetPriority() const = 0;

        //! Compare function required by BusHandlerOrderCompare = BusHandlerCompareDefault
        //! \param[in] other Another instance of the class to compare
        //! @return True if this bus should be processed before the other.
        inline bool Compare(const PC2PostProcessingRequests* other) const
        {
            return GetPriority() > other->GetPriority();
        }
    };

    using PC2PostProcessingRequestBus = AZ::EBus<PC2PostProcessingRequests>;
} // namespace ROS2
