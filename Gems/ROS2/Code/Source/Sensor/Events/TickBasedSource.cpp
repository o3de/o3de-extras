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
        float expectedSimulationLoopTime = -1.f;
        // query time ROS2 system
        ROS2ClockRequestBus::BroadcastResult(
            expectedSimulationLoopTime, &ROS2ClockRequestBus::Events::GetExpectedLoopTime);
        AZ_Assert(expectedSimulationLoopTime>= 0.f, "Did not receive expected simulation loop time from ROS2ClockRequestBus");
        m_sourceEvent.Signal(expectedSimulationLoopTime);
    }
} // namespace ROS2
