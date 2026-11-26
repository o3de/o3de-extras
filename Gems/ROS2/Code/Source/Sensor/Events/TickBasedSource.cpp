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

    void TickBasedSource::SetPacing(SensorPacing pacing)
    {
        m_pacing = pacing;
    }

    void TickBasedSource::OnTick(float deltaTime, AZ::ScriptTimePoint time)
    {
        AZ_UNUSED(time);
        AZ_UNUSED(deltaTime);
        float expectedSimulationLoopTime = -1.f;
        // query time ROS2 system
        ROS2ClockRequestBus::BroadcastResult(
            expectedSimulationLoopTime, &ROS2ClockRequestBus::Events::GetExpectedLoopTime);
        AZ_Warning("TickBasedSource", expectedSimulationLoopTime >= 0.f, "Did not receive expected simulation loop time from ROS2ClockRequestBus");
        if (m_pacing == SensorPacing::Frames)
        {
            m_sourceEvent.Signal(expectedSimulationLoopTime);
            return;
        }

        // SensorPacing::SimulationTime: the ROS time since the previous tick. The first tick has no previous time and uses the expected
        // loop time.
        builtin_interfaces::msg::Time now;
        ROS2ClockRequestBus::BroadcastResult(now, &ROS2ClockRequestBus::Events::GetROSTimestamp);
        const bool hasPreviousTick = m_lastTickTimestamp.sec != 0 || m_lastTickTimestamp.nanosec != 0;
        const float simulationDeltaTime = hasPreviousTick ? ROS2Conversions::GetTimeDifference(m_lastTickTimestamp, now)
                                                          : expectedSimulationLoopTime;
        m_lastTickTimestamp = now;
        m_sourceEvent.Signal(AZStd::max(0.f, simulationDeltaTime));
    }
} // namespace ROS2
