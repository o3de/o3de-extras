/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <CpuProfiler.h>

#include <AzCore/Debug/ProfilerBus.h>
#include <AzCore/Interface/Interface.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/Utils/Utils.h>
#include <AzCore/std/time.h>
#include <optick.h>

namespace OptickProfiler
{
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

    void CpuProfiler::Init()
    {
        AZ::Interface<AZ::Debug::Profiler>::Register(this);
        AZ::SystemTickBus::Handler::BusConnect();

        Optick::SetStateChangedCallback(OnOptickStateChanged);
        m_initialized = true;
    }

    void CpuProfiler::Shutdown()
    {
        if (!m_initialized)
        {
            return;
        }

        // When this call is made, no more thread profiling calls can be performed anymore
        AZ::Interface<AZ::Debug::Profiler>::Unregister(this);

        // Wait for the remaining threads that might still be processing its profiling calls
        AZStd::unique_lock<AZStd::shared_mutex> shutdownLock(m_shutdownMutex);

        AZ::SystemTickBus::Handler::BusDisconnect();

        Optick::StopCapture();
        Optick::Shutdown();
    }

    void CpuProfiler::BeginRegion([[maybe_unused]] const AZ::Debug::Budget* budget, [[maybe_unused]] const char* eventName, ...)
    {
        // Try to lock here, the shutdownMutex will only be contested when the CpuProfiler is shutting down.
        if (m_shutdownMutex.try_lock_shared())
        {
            Optick::Event::Push(eventName);
            m_shutdownMutex.unlock_shared();
        }
    }

    void CpuProfiler::EndRegion([[maybe_unused]] const AZ::Debug::Budget* budget)
    {
        // Try to lock here, the shutdownMutex will only be contested when the CpuProfiler is shutting down.
        if (m_shutdownMutex.try_lock_shared())
        {
            Optick::Event::Pop();
            m_shutdownMutex.unlock_shared();
        }
    }

    void CpuProfiler::OnSystemTick()
    {
        Optick::Update();
    }
} // namespace OptickProfiler
