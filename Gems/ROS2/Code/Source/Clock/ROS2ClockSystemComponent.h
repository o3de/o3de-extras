/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Component/Component.h>
#include "ITimeSource.h"
#include <AzCore/Outcome/Outcome.h>
#include <AzCore/std/smart_ptr/unique_ptr.h>
#include <AzCore/std/string/string.h>
#include <rclcpp/publisher.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <ROS2/Clock/ROS2ClockRequestBus.h>
#include <AzCore/Component/TickBus.h>

namespace ROS2
{
    //! The ROS2Clock provides ROS timestamps as builtin_interface::msg::Time messages.
    //! The clock can use different types of the time sources and publish the current
    //! time to the ROS 2 `/clock/` topic. The published time can be used with
    //! the /use_sim_time parameter set to true.
    class ROS2ClockSystemComponent :
        public AZ::Component,
        public AZ::TickBus::Handler,
        public ROS2ClockRequestBus::Handler
    {
    public:
        AZ_COMPONENT_DECL(ROS2ClockSystemComponent);

        static void Reflect(AZ::ReflectContext* context);

        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);

        ROS2ClockSystemComponent() = default;
        ~ROS2ClockSystemComponent() override = default;

        // AZ::Component overrides
        void Activate();
        void Deactivate();

        // AZ::TickBus overrides
        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;

        // ROS2ClockRequestBus overrides
        builtin_interfaces::msg::Time GetROSTimestamp() const override;
        double GetROSTimestampSec() const override;
        AZ::Outcome<void, AZStd::string> AdjustTime(const builtin_interfaces::msg::Time& time) override;
        bool AdjustTimeDouble(double time) override;
        void PublishTime() override;
        float GetExpectedLoopTime() const override;

    private:
        rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr m_clockPublisher;
        AZStd::unique_ptr<ITimeSource> m_timeSource; //! Source Clock implementation

        AZStd::deque<float> m_simulationLoopTimes;
        builtin_interfaces::msg::Time m_lastSimulationTime;
        float m_simulationLoopTimeMedian = 1.0f / 60.0f;

        bool m_publishClock;
    };
} // namespace ROS2
