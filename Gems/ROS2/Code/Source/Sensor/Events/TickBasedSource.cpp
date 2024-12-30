/*
* Copyright (c) Contributors to the Open 3D Engine Project.
* For complete copyright and license terms please see the LICENSE at the root of this distribution.
*
* SPDX-License-Identifier: Apache-2.0 OR MIT
*
*/

#include <ROS2/Sensor/Events/TickBasedSource.h>
#include <ROS2/Sensor/SensorConfiguration.h>
#include <ROS2/ROS2Bus.h>
#include <ROS2/Utilities/ROS2Conversions.h>
#include <AzCore/std/numeric.h>
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
        const auto simTimestamp = ROS2Interface::Get()->GetExpectedSimulationLoopTime();
        m_sourceEvent.Signal(simTimestamp);

    }
} // namespace ROS2
