/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/EBus/EBus.h>
#include <AzCore/EBus/Event.h>
#include <AzCore/Interface/Interface.h>
#include <ROS2/Clock/SimulationClock.h>
#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/node.hpp>

namespace ROS2
{
    //! Interface to the central ROS2SystemComponent.
    //! Use this API through ROS2Interface, for example:
    //! @code
    //! auto node = ROS2Interface::Get()->GetNode();
    //! @endcode
    class ROS2Requests
    {
    public:
        using NodeChangedEvent = AZ::Event<std::shared_ptr<rclcpp::Node>>;

        AZ_RTTI(ROS2Requests, "{a9bdbff6-e644-430d-8096-cdb53c88e8fc}");
        virtual ~ROS2Requests() = default;

        //! Get a central ROS2 node of the Gem.
        //! You can use this node to create publishers and subscribers.
        //! @return The central ROS2 node which holds default publishers for core topics such as /clock and /tf.
        //! @note Alternatively, you can use your own node along with an executor.
        virtual std::shared_ptr<rclcpp::Node> GetNode() const = 0;

        //! Attach handler to ROS2 node change events.
        //! You can use this method to correctly initialize SystemComponents that depend on ROS2Node.
        //! @param handler which will be connected to NodeChangedEvent.
        //! @note callback is active as long as handler is not destroyed.
        virtual void ConnectOnNodeChanged(NodeChangedEvent::Handler& handler) = 0;

        //! Acquire current time as ROS2 timestamp.
        //! Timestamps provide temporal context for messages such as sensor data.
        //! @code
        //! auto message = sensor_msgs::msg::PointCloud2();
        //! message.header.stamp = ROS2Interface::Get()->GetROSTimestamp();
        //! @endcode
        //! @return Simulation time in ROS2 format. Time source is also valid with non-real time settings.
        //! @note Make sure to set the use_sim_time parameter for ROS2 nodes which will use the simulation data.
        virtual builtin_interfaces::msg::Time GetROSTimestamp() const = 0;

        //! Send transformation between ROS2 frames.
        //! @param t is a <a href="https://docs.ros2.org/latest/api/geometry_msgs/msg/TransformStamped.html">ROS2 TransformStamped
        //! message</a>.
        //! @param isDynamic controls whether a static or dynamic transform is sent. Static transforms are published
        //! only once and are to be used when the spatial relationship between two frames does not change.
        //! @note Transforms are already published by each ROS2FrameComponent.
        //! Use this function directly only when default behavior of ROS2FrameComponent is not sufficient.
        virtual void BroadcastTransform(const geometry_msgs::msg::TransformStamped& t, bool isDynamic) = 0;

        //! Obtains a simulation clock that is used across simulation.
        //! @returns constant reference to currently running clock.
        virtual const SimulationClock& GetSimulationClock() const = 0;
    };

    class ROS2BusTraits : public AZ::EBusTraits
    {
    public:
        //////////////////////////////////////////////////////////////////////////
        // EBusTraits overrides
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::Single;
        //////////////////////////////////////////////////////////////////////////
    };

    using ROS2RequestBus = AZ::EBus<ROS2Requests, ROS2BusTraits>;
    using ROS2Interface = AZ::Interface<ROS2Requests>;
} // namespace ROS2
