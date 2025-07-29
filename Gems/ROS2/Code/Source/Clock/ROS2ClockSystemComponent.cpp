/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ROS2ClockSystemComponent.h"
#include "ROS2TimeSource.h"
#include "RealTimeSource.h"
#include "SimulationTimeSource.h"
#include <AzCore/RTTI/BehaviorContext.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/Settings/SettingsRegistry.h>
#include <AzCore/std/sort.h>
#include <ROS2/ROS2Bus.h>
#include <ROS2/Utilities/ROS2Conversions.h>
#include <rclcpp/qos.hpp>

namespace ROS2
{
    constexpr AZStd::string_view ClockTypeConfigurationKey = "/O3DE/ROS2/ClockType";
    constexpr AZStd::string_view PublishClockConfigurationKey = "/O3DE/ROS2/PublishClock";
    constexpr AZStd::string_view RealtimeClockName = "realtime";
    constexpr AZStd::string_view SimulationClockName = "simulation";
    constexpr AZStd::string_view Ros2ClockName = "ros2";
    constexpr AZStd::string_view DefaultClock = SimulationClockName;

    constexpr size_t FramesNumberForStats = 60;

    AZ_COMPONENT_IMPL(ROS2ClockSystemComponent, "ROS2ClockSystemComponent", ROS2ClockSystemComponentTypeId);

    void ROS2ClockSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<ROS2ClockSystemComponent, AZ::Component>()->Version(0);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<ROS2ClockSystemComponent>("ROS 2 Clock System Component", "This component provides ROS 2 clock functionality.")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("System"))
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->Attribute(AZ::Edit::Attributes::AutoExpand, true);
            }
        }
        if (AZ::BehaviorContext* behaviorContext = azrtti_cast<AZ::BehaviorContext*>(context))
        {
            behaviorContext
                ->EBus<ROS2ClockRequestBus>("ROS2ClockRequestBus")

                ->Event("GetRosTimestampSec", &ROS2ClockRequestBus::Events::GetROSTimestampSec)
                ->Event("PublishTime", &ROS2ClockRequestBus::Events::PublishTime)
                ->Event("GetExpectedLoopTime", &ROS2ClockRequestBus::Events::GetExpectedLoopTime)
                ->Event("AdjustTimeSec", &ROS2ClockRequestBus::Events::AdjustTimeSec);
        }
    }

    ROS2ClockSystemComponent::ROS2ClockSystemComponent()
    {
        if (ROS2ClockInterface::Get() == nullptr)
        {
            ROS2ClockInterface::Register(this);
        }
    }

    ROS2ClockSystemComponent::~ROS2ClockSystemComponent()
    {
        if (ROS2ClockInterface::Get() == this)
        {
            ROS2ClockInterface::Unregister(this);
        }
    }

    void ROS2ClockSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("ROS2ClockService"));
    }

    void ROS2ClockSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("ROS2ClockService"));
    }

    void ROS2ClockSystemComponent::Activate()
    {
        // Read configuration from the settings registry
        AZStd::unordered_map<AZStd::string, AZStd::function<AZStd::unique_ptr<ITimeSource>()>> clocksMap;
        clocksMap[RealtimeClockName] = []()
        {
            AZ_Info("ROS2ClockSystemComponent", "Enabling realtime clock.\n");
            return AZStd::make_unique<RealTimeSource>();
        };
        clocksMap[SimulationClockName] = []()
        {
            AZ_Info("ROS2ClockSystemComponent", "Enabling simulation clock.\n");
            return AZStd::make_unique<SimulationTimeSource>();
        };
        clocksMap[Ros2ClockName] = []()
        {
            AZ_Info("ROS2ClockSystemComponent", "Enabling ros 2 clock.\n");
            return AZStd::make_unique<ROS2TimeSource>();
        };

        AZStd::string clockType{ "" };
        bool publishClock{ true };
        auto* registry = AZ::SettingsRegistry::Get();
        AZ_Assert(registry, "No Registry available");
        if (registry)
        {
            registry->Get(publishClock, PublishClockConfigurationKey);
            registry->Get(clockType, ClockTypeConfigurationKey);
            if (clocksMap.contains(clockType))
            {
                m_timeSource = clocksMap[clockType]();
                m_publishClock = publishClock;
            }
        }

        if (!m_timeSource)
        {
            AZ_Info(
                "ROS2SystemComponent",
                "Cannot read registry or the clock type '%s' is unknown, enabling %s clock.\n",
                clockType.c_str(),
                DefaultClock.data());
            m_timeSource = clocksMap[DefaultClock]();
            m_publishClock = true;
        }

        m_timeSource->Activate();
        m_simulationLoopTimes.clear();
        AZ::TickBus::Handler::BusConnect();
        ROS2ClockRequestBus::Handler::BusConnect();
    }

    void ROS2ClockSystemComponent::Deactivate()
    {
        ROS2ClockRequestBus::Handler::BusDisconnect();
        AZ::TickBus::Handler::BusDisconnect();
        // Deactivate the time source
        if (m_timeSource)
        {
            m_timeSource->Deactivate();
        }
        m_clockPublisher.reset();
    }

    builtin_interfaces::msg::Time ROS2ClockSystemComponent::GetROSTimestamp() const
    {
        AZ_Assert(m_timeSource, "No time source set for ROS2ClockSystemComponent");
        return m_timeSource->GetROSTimestamp();
    }

    double ROS2ClockSystemComponent::GetROSTimestampSec() const
    {
        AZ_Assert(m_timeSource, "No time source set for ROS2ClockSystemComponent");
        builtin_interfaces::msg::Time timeStamp = m_timeSource->GetROSTimestamp();
        return double(timeStamp.sec) + double(timeStamp.nanosec) * 1e-9;
    }

    AZ::Outcome<void, AZStd::string> ROS2ClockSystemComponent::AdjustTime(const builtin_interfaces::msg::Time& time)
    {
        AZ_Assert(m_timeSource, "No time source set for ROS2ClockSystemComponent");
        return m_timeSource->AdjustTime(time);
    }

    bool ROS2ClockSystemComponent::AdjustTimeDouble(double time)
    {
        AZ_Assert(m_timeSource, "No time source set for ROS2ClockSystemComponent");
        builtin_interfaces::msg::Time timeStamp;
        timeStamp.sec = static_cast<int32_t>(time);
        timeStamp.nanosec = static_cast<uint32_t>((time - timeStamp.sec) * 1e9);
        const auto result = m_timeSource->AdjustTime(timeStamp);
        if (!result.IsSuccess())
        {
            AZ_Error("ROS2ClockSystemComponent", false, "Failed to adjust time: %s", result.GetError().c_str());
        }
        return result.IsSuccess();
    }

    void ROS2ClockSystemComponent::PublishTime()
    {
        if (!m_publishClock)
        {
            return;
        }
        if (!m_clockPublisher)
        {
            // Lazy construct
            auto ros2Node = ROS2Interface::Get()->GetNode();
            if (ros2Node)
            {
                rclcpp::ClockQoS qos;
                m_clockPublisher = ros2Node->create_publisher<rosgraph_msgs::msg::Clock>("/clock", qos);
            }
        }

        rosgraph_msgs::msg::Clock msg;
        msg.clock = GetROSTimestamp();
        m_clockPublisher->publish(msg);
    }

    float ROS2ClockSystemComponent::GetExpectedLoopTime() const
    {
        return m_simulationLoopTimeMedian;
    }

    void ROS2ClockSystemComponent::OnTick(float deltaTime, AZ::ScriptTimePoint time)
    {
        PublishTime();
        // Calculate simulation loop time
        const auto simulationTimestamp = GetROSTimestamp();
        const float deltaSimTime = ROS2Conversions::GetTimeDifference(m_lastSimulationTime, simulationTimestamp);

        // Filter out the outliers
        m_simulationLoopTimes.push_back(deltaSimTime);
        if (m_simulationLoopTimes.size() > FramesNumberForStats)
        {
            m_simulationLoopTimes.pop_front();
        }
        AZStd::vector<float> frameTimeSorted{ m_simulationLoopTimes.begin(), m_simulationLoopTimes.end() };
        AZStd::sort(frameTimeSorted.begin(), frameTimeSorted.end());
        m_simulationLoopTimeMedian = frameTimeSorted[frameTimeSorted.size() / 2];

        m_lastSimulationTime = simulationTimestamp;
    }
} // namespace ROS2
