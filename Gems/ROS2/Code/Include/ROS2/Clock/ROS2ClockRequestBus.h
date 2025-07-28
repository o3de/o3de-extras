/*
* Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Component/ComponentBus.h>
#include <AzCore/Outcome/Outcome.h>
#include <AzCore/Component/EntityId.h>
#include <AzCore/EBus/EBus.h>
#include <ROS2/ROS2TypeIds.h>


namespace ROS2
{
    //! Interface that allows to get sensor configuration and switch publish state.
    class ROS2ClockRequests : public AZ::EBusTraits
    {
    public:
        AZ_RTTI(ROS2ClockRequests, ROS2ClockRequestsTypeId);

        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::Single;
        static const AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;

        virtual ~ROS2ClockRequests() = default;

        //! Sets the time source to the given time.
        //! @param time The time to set the time source to.
        //! @return An outcome indicating success or failure.
        virtual AZ::Outcome<void, AZStd::string> AdjustTime(const builtin_interfaces::msg::Time& time) = 0;

        //! Sets the time source to the given time as a double value.
        //! @param time The time to set the time source to, in seconds.
        //! @return true if the time was adjusted successfully, false otherwise.
        virtual bool AdjustTimeDouble(double time) = 0;

        //! Acquire current time as ROS2 timestamp.
        //! Timestamps provide temporal context for messages such as sensor data.
        //! @code
        //! auto message = sensor_msgs::msg::PointCloud2();
        //! ROS2ClockRequestBus::BroadcastResult(
        //!        message.header.stamp, &ROS2ClockRequestBus::Events::GetROSTimestamp);
        //! @endcode
        //! @return Simulation time in ROS2 format.
        //! @note Make sure to set the use_sim_time parameter for ROS2 nodes which will use the simulation data.
        virtual builtin_interfaces::msg::Time GetROSTimestamp() const = 0;

        //! Get the current ROS timestamp in seconds, returns the GetROSTimestamp as a double value.
        virtual double GetROSTimestampSec() const = 0;

        //! Force publishing of timestamp to the `/clock` topic.
        //! This is useful when the clock is not automatically published, for example in simulation mode.
        virtual void PublishTime() = 0;

        //! Returns the expected time in seconds that the simulation loop (time between two frames) takes to execute.
        //! This is useful for triggering sensor
        virtual float GetExpectedLoopTime() const = 0;
    };

    using ROS2ClockRequestBus = AZ::EBus<ROS2ClockRequests>;
} // namespace ROS2