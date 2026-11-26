/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/Component/TickBus.h>
#include <AzCore/EBus/Event.h>
#include <ROS2/ROS2Bus.h>
#include <ROS2/ROS2TypeIds.h>
#include <ROS2/Sensor/Events/SensorEventSource.h>
#include <ROS2/Sensor/Events/SensorPacing.h>

namespace ROS2
{
    //! Class implementing system TickBus (draw calls) as sensor event source. Source event (ROS2::SensorEventSource) is signalled based on
    //! system ticks.
    //! @see ROS2::SensorEventSource
    class TickBasedSource final
        : public SensorEventSource<AZ::Event, AZ::EventHandler, float>
        , protected AZ::TickBus::Handler
    {
    public:
        AZ_TYPE_INFO(TickBasedSource, TickBasedSourceTypeId);
        static void Reflect(AZ::ReflectContext* context);

        // Overrides of ROS2::SensorEventSource.
        void Start() override;
        void Stop() override;
        [[nodiscard]] float GetDeltaTime(float deltaTime) const override;

        //! SensorPacing::Frames (default): signal the expected simulation loop time, the median frame time.
        //! SensorPacing::SimulationTime: signal the ROS time since the previous tick. ROS2::EventSourceAdapter sets it on Start.
        void SetPacing(SensorPacing pacing);

    private:
        // Override of AZ::TickBus::Handler.
        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;

        SensorPacing m_pacing{ SensorPacing::Frames };
        builtin_interfaces::msg::Time m_lastTickTimestamp;
        AZStd::deque<float> m_deltaTimes;
        constexpr static size_t m_maxDeltaTimes = 100;
    };
} // namespace ROS2
