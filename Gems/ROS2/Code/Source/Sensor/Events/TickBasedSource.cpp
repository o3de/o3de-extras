/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/std/numeric.h>
#include <ROS2/ROS2Bus.h>
#include <ROS2/Sensor/Events/TickBasedSource.h>
#include <ROS2/Utilities/ROS2Conversions.h>
#include <ROS2/Clock/ROS2ClockRequestBus.h>
namespace ROS2
{
    void TickBasedSource::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<TickBasedSource>()->Version(1);
        }
    }

    void TickBasedSource::Start()
    {
        m_lastTickTimestamp = builtin_interfaces::msg::Time();
        AZ::TickBus::Handler::BusConnect();
    }

    void TickBasedSource::Stop()
    {
        AZ::TickBus::Handler::BusDisconnect();
    }

    float TickBasedSource::GetDeltaTime(float deltaTime) const
    {
        return deltaTime;
    }

    void TickBasedSource::OnTick(float deltaTime, AZ::ScriptTimePoint time)
    {
        AZ_UNUSED(time);
        AZ_UNUSED(deltaTime);
        // Signal the ROS time that passed since the last tick, so that adapters pace sensors on simulation time. The first tick has
        // no previous time; it uses the expected loop time, as the median frame time is used when nothing else is known.
        builtin_interfaces::msg::Time now;
        ROS2ClockRequestBus::BroadcastResult(now, &ROS2ClockRequestBus::Events::GetROSTimestamp);
        float simulationDeltaTime = -1.f;
        if (m_lastTickTimestamp.sec != 0 || m_lastTickTimestamp.nanosec != 0)
        {
            simulationDeltaTime = AZStd::max(0.f, ROS2Conversions::GetTimeDifference(m_lastTickTimestamp, now));
        }
        else
        {
            ROS2ClockRequestBus::BroadcastResult(simulationDeltaTime, &ROS2ClockRequestBus::Events::GetExpectedLoopTime);
            AZ_Warning(
                "TickBasedSource", simulationDeltaTime >= 0.f, "Did not receive expected simulation loop time from ROS2ClockRequestBus");
            simulationDeltaTime = AZStd::max(0.f, simulationDeltaTime);
        }
        m_lastTickTimestamp = now;
        m_sourceEvent.Signal(simulationDeltaTime);
    }
} // namespace ROS2
