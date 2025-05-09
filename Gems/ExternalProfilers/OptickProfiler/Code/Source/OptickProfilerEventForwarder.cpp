/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <OptickProfilerEventForwarder.h>

#include <AzCore/Debug/ProfilerBus.h>
#include <AzCore/Interface/Interface.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/Utils/Utils.h>
#include <AzCore/std/parallel/thread.h>
#include <AzCore/std/time.h>
#include <optick.h>

namespace OptickProfiler
{
    thread_local Optick::EventStorage* OptickProfilerEventForwarder::m_pOptickStorage = nullptr;

    bool OnOptickStateChanged(Optick::State::Type state)
    {
        switch (state)
        {
        case Optick::State::DUMP_CAPTURE:
            {
                auto projectName = AZ::Utils::GetProjectName();
                Optick::AttachSummary("Project", projectName.c_str());
            }
            break;
        }
        return true;
    }

    void OptickProfilerEventForwarder::Init()
    {
        AZ::Interface<AZ::Debug::Profiler>::Register(this);
        AZ::TickBus::Handler::BusConnect();
        AZStd::ThreadEventBus::Handler::BusConnect();

        Optick::SetStateChangedCallback(OnOptickStateChanged);
        m_pOptickFrameTag = Optick::EventDescription::Create("Frame", "", 0);
        m_pOptickStorage = Optick::RegisterStorage("Main Thread", AZStd::this_thread::get_id().m_id);

        m_initialized = true;
    }

    void OptickProfilerEventForwarder::Shutdown()
    {
        if (!m_initialized)
        {
            return;
        }

        // When this call is made, no more thread profiling calls can be performed anymore
        AZ::Interface<AZ::Debug::Profiler>::Unregister(this);

        // Wait for the remaining threads that might still be processing its profiling calls
        AZStd::unique_lock<AZStd::shared_mutex> shutdownLock(m_shutdownMutex);

        AZStd::ThreadEventBus::Handler::BusDisconnect();
        AZ::TickBus::Handler::BusDisconnect();

        Optick::StopCapture();
        Optick::Shutdown();
    }

    void OptickProfilerEventForwarder::BeginRegion(
        [[maybe_unused]] const AZ::Debug::Budget* budget, [[maybe_unused]] const char* eventName, ...)
    {
        if (!m_pOptickStorage && m_threadIdToNameMutex.try_lock_shared())
        {
            AZStd::native_thread_id_type id = AZStd::this_thread::get_id().m_id;
            const AZStd::string& name = m_threadIdToName.contains(id) ? m_threadIdToName.at(id) : "";

            m_pOptickStorage = Optick::RegisterStorage(name.c_str(), id);
            m_threadIdToNameMutex.unlock_shared();
        }

        // Try to lock here, the shutdownMutex will only be contested when the CpuProfiler is shutting down.
        if (m_shutdownMutex.try_lock_shared())
        {
            Optick::EventDescription* pDesc = Optick::EventDescription::CreateShared(eventName);
            Optick::Event::Push(m_pOptickStorage, pDesc, Optick::GetHighPrecisionTime());
            m_shutdownMutex.unlock_shared();
        }
    }

    void OptickProfilerEventForwarder::EndRegion([[maybe_unused]] const AZ::Debug::Budget* budget)
    {
        // Try to lock here, the shutdownMutex will only be contested when the CpuProfiler is shutting down.
        if (m_shutdownMutex.try_lock_shared())
        {
            Optick::Event::Pop(m_pOptickStorage, Optick::GetHighPrecisionTime());
            m_shutdownMutex.unlock_shared();
        }
    }

    int OptickProfilerEventForwarder::GetTickOrder()
    {
        return AZ::ComponentTickBus::TICK_FIRST;
    }

    void OptickProfilerEventForwarder::OnTick([[maybe_unused]] float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint timePoint)
    {
        const int64_t frameTick = Optick::GetHighPrecisionTime();

        Optick::Event::Pop(m_pOptickStorage, frameTick);
        Optick::EndFrame(Optick::FrameType::CPU);

        Optick::Update();

        const uint32_t frameNumber = Optick::BeginFrame(Optick::FrameType::CPU, frameTick);
        Optick::Event::Push(m_pOptickStorage, Optick::GetFrameDescription(), frameTick);
        Optick::AttachTag(m_pOptickStorage, *m_pOptickFrameTag, frameNumber, frameTick);
    }

    void OptickProfilerEventForwarder::OnThreadEnter(const AZStd::thread::id& id, const AZStd::thread_desc* desc)
    {
        AZStd::unique_lock<AZStd::shared_mutex> lock(m_threadIdToNameMutex);
        m_threadIdToName[id.m_id] = desc->m_name;
    }

    void OptickProfilerEventForwarder::OnThreadExit([[maybe_unused]] const AZStd::thread::id& id)
    {
    }
} // namespace OptickProfiler
