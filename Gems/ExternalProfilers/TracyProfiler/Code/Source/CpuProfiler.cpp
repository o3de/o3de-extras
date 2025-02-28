/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <CpuProfiler.h>

#include <AzCore/Interface/Interface.h>
#include <AzCore/Serialization/SerializeContext.h>

namespace TracyProfiler
{
    thread_local CpuProfiler::EventIdStack CpuProfiler::ms_threadLocalStorage;

    void CpuProfiler::Init()
    {
        AZ::Interface<AZ::Debug::Profiler>::Register(this);
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
    }

    void CpuProfiler::BeginRegion(const AZ::Debug::Budget* budget, const char* eventName, ...)
    {
        // Try to lock here, the shutdownMutex will only be contested when the CpuProfiler is shutting down.
        if (m_shutdownMutex.try_lock_shared())
        {
            // Do not use the macro as we don't want to use static storage (else events are wrong)
            ms_threadLocalStorage.push_back({});
            TracyCZoneCtx& ctx = ms_threadLocalStorage.back();
            ctx = ___tracy_emit_zone_begin_alloc_callstack(
                ___tracy_alloc_srcloc_name(
                    __LINE__, __FILE__, strlen(__FILE__), __FUNCTION__, strlen(__FUNCTION__), eventName, strlen(eventName), budget->Crc()),
                TRACY_CALLSTACK,
                true);
            TracyCZoneText(ctx, budget->Name(), 1);
            m_shutdownMutex.unlock_shared();
        }
    }

    void CpuProfiler::EndRegion([[maybe_unused]] const AZ::Debug::Budget* budget)
    {
        // Try to lock here, the shutdownMutex will only be contested when the CpuProfiler is shutting down.
        if (m_shutdownMutex.try_lock_shared() && !ms_threadLocalStorage.empty())
        {
            auto& ctx = ms_threadLocalStorage.back();
            TracyCZoneEnd(ctx);
            ms_threadLocalStorage.pop_back();

            m_shutdownMutex.unlock_shared();
        }
    }
} // namespace TracyProfiler
