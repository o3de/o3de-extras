/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <AzCore/EBus/Event.h>
#include <AzCore/RTTI/TypeInfo.h>
#include <AzCore/std/containers/vector.h>
#include <ROS2/Sensor/Events/EventSourceAdapter.h>
#include <ROS2/Sensor/Events/SensorEventSource.h>
#include <cmath>
#include <gtest/gtest.h>

namespace UnitTest
{
    //! Event source driven by the test: every Tick(deltaTime) signals the source event with that delta time. The ticks come from an
    //! event that the test fixture owns, so that nothing allocated outlives the test environment.
    class ManualEventSource final : public ROS2::SensorEventSource<AZ::Event, AZ::EventHandler, float>
    {
    public:
        AZ_TYPE_INFO(ManualEventSource, "{5B1D8C0A-3E4F-4A7B-9C2D-6E8F1A2B3C4D}");

        static void Reflect([[maybe_unused]] AZ::ReflectContext* context)
        {
        }

        static inline AZ::Event<float>* s_ticks = nullptr;

        static void Tick(float deltaTime)
        {
            s_ticks->Signal(deltaTime);
        }

        void Start() override
        {
            m_tickHandler = AZ::Event<float>::Handler(
                [this](float deltaTime)
                {
                    m_sourceEvent.Signal(deltaTime);
                });
            m_tickHandler.Connect(*s_ticks);
        }

        void Stop() override
        {
            m_tickHandler.Disconnect();
        }

        [[nodiscard]] float GetDeltaTime(float deltaTime) const override
        {
            return deltaTime;
        }

    private:
        AZ::Event<float>::Handler m_tickHandler;
    };

    class EventSourceAdapterTest : public ::testing::Test
    {
    protected:
        void SetUp() override
        {
            ManualEventSource::s_ticks = &m_ticks;
        }

        void Start(float frequency)
        {
            m_adapter.SetFrequency(frequency);
            m_adaptedHandler = ManualEventSource::AdaptedEventHandlerType(
                [this](float adaptedDeltaTime, [[maybe_unused]] float sourceDeltaTime)
                {
                    m_adaptedDeltaTimes.push_back(adaptedDeltaTime);
                });
            m_adapter.ConnectToAdaptedEvent(m_adaptedHandler);
            m_adapter.Start();
        }

        void TearDown() override
        {
            m_adapter.Stop();
            m_adaptedHandler.Disconnect();
            ManualEventSource::s_ticks = nullptr;
        }

        //! Ticks at frameRate for the given time, with the delta times quantized to clockStep, as when ROS time comes from a /clock
        //! topic published in clockStep steps. Returns the simulated time that passed.
        static double TickQuantized(double frameRate, double clockStep, double seconds)
        {
            double previousClock = 0.0;
            const int frames = static_cast<int>(std::lround(seconds * frameRate));
            for (int frame = 1; frame <= frames; ++frame)
            {
                const double clock = std::floor(frame / frameRate / clockStep + 1e-9) * clockStep;
                ManualEventSource::Tick(static_cast<float>(clock - previousClock));
                previousClock = clock;
            }
            return previousClock;
        }

        AZ::Event<float> m_ticks;
        ROS2::EventSourceAdapter<ManualEventSource> m_adapter;
        ManualEventSource::AdaptedEventHandlerType m_adaptedHandler;
        AZStd::vector<float> m_adaptedDeltaTimes;
    };

    TEST_F(EventSourceAdapterTest, KeepsTheFrequencyWhenTheSourceIsNotAnIntegerMultiple)
    {
        Start(10.0f);
        TickQuantized(103.5, 1e-6, 60.0);
        EXPECT_NEAR(m_adaptedDeltaTimes.size(), 600, 1);
    }

    TEST_F(EventSourceAdapterTest, KeepsTheFrequencyWithQuantizedDeltaTimes)
    {
        // 160 FPS with ROS time from a 400 Hz /clock: the delta times alternate between 5 and 7.5 ms. Counting
        // round(source rate / frequency) frames published 8 Hz here.
        Start(10.0f);
        TickQuantized(160.0, 0.0025, 60.0);
        EXPECT_NEAR(m_adaptedDeltaTimes.size(), 600, 1);
    }

    TEST_F(EventSourceAdapterTest, AdaptedDeltaTimesAddUpToTheSourceTime)
    {
        Start(10.0f);
        const double elapsed = TickQuantized(94.0, 0.0025, 30.0);
        double adapted = 0.0;
        for (const float deltaTime : m_adaptedDeltaTimes)
        {
            adapted += deltaTime;
        }
        // Everything but the time since the last publication, which is less than one period.
        EXPECT_GE(adapted, elapsed - 0.1 - 1e-3);
        EXPECT_LE(adapted, elapsed + 1e-3);
    }

    TEST_F(EventSourceAdapterTest, SignalsEverySourceEventWhenTheFrequencyIsHigher)
    {
        Start(100.0f);
        for (int frame = 0; frame < 30; ++frame)
        {
            ManualEventSource::Tick(1.0f / 30.0f);
        }
        EXPECT_EQ(m_adaptedDeltaTimes.size(), 30);
    }

    TEST_F(EventSourceAdapterTest, DropsTheBacklogAfterAStall)
    {
        Start(10.0f);
        for (int frame = 0; frame < 100; ++frame)
        {
            ManualEventSource::Tick(0.01f);
        }
        const size_t beforeStall = m_adaptedDeltaTimes.size();
        ManualEventSource::Tick(1.0f);
        EXPECT_EQ(m_adaptedDeltaTimes.size(), beforeStall + 1);
        for (int frame = 0; frame < 9; ++frame)
        {
            ManualEventSource::Tick(0.01f);
        }
        // No burst to catch up on the stalled second: the next publication is one period after the stall.
        EXPECT_EQ(m_adaptedDeltaTimes.size(), beforeStall + 1);
        ManualEventSource::Tick(0.01f);
        EXPECT_EQ(m_adaptedDeltaTimes.size(), beforeStall + 2);
    }
} // namespace UnitTest
