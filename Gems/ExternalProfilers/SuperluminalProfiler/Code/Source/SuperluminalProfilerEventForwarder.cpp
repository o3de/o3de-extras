/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <SuperluminalProfilerEventForwarder.h>

#include <AzCore/Interface/Interface.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <Superluminal/PerformanceAPI.h>

namespace SuperluminalProfiler
{
    void SuperluminalProfilerEventForwarder::Init()
    {
        AZ::Interface<AZ::Debug::Profiler>::Register(this);
        m_initialized = true;
    }

    void SuperluminalProfilerEventForwarder::Shutdown()
    {
        if (!m_initialized)
        {
            return;
        }

        // When this call is made, no more thread profiling calls can be performed anymore
        AZ::Interface<AZ::Debug::Profiler>::Unregister(this);

        // Wait for the remaining threads that might still be processing its profiling calls
        AZStd::unique_lock<AZStd::shared_mutex> shutdownLock(m_shutdownMutex);
    }

    void SuperluminalProfilerEventForwarder::BeginRegion(const AZ::Debug::Budget* budget, const char* eventName, ...)
    {
        // Try to lock here, the shutdownMutex will only be contested when the CpuProfiler is shutting down.
        if (m_shutdownMutex.try_lock_shared())
        {
            PerformanceAPI_BeginEvent(eventName, budget->Name(), budget->Crc());
            m_shutdownMutex.unlock_shared();
        }
    }

    void SuperluminalProfilerEventForwarder::EndRegion([[maybe_unused]] const AZ::Debug::Budget* budget)
    {
        // Try to lock here, the shutdownMutex will only be contested when the CpuProfiler is shutting down.
        if (m_shutdownMutex.try_lock_shared())
        {
            PerformanceAPI_EndEvent();
            m_shutdownMutex.unlock_shared();
        }
    }
} // namespace SuperluminalProfiler
