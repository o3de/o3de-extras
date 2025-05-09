/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/Component/TickBus.h>
#include <AzCore/Debug/Profiler.h>
#include <AzCore/Memory/SystemAllocator.h>
#include <AzCore/Name/Name.h>
#include <AzCore/RTTI/RTTI.h>
#include <AzCore/std/containers/vector.h>
#include <AzCore/std/parallel/mutex.h>
#include <AzCore/std/parallel/shared_mutex.h>
#include <tracy/TracyC.h>

namespace TracyProfiler
{
    //! Listen to 03DE frame/profiling events and forward them to the Tracy profiling library
    class TracyProfilerEventForwarder final
        : public AZ::Debug::Profiler
        , public AZ::TickBus::Handler
    {
    public:
        AZ_RTTI(TracyProfilerEventForwarder, "{9467E3F6-0581-4E46-A98A-F3C249FD7B24}", AZ::Debug::Profiler);
        AZ_CLASS_ALLOCATOR(TracyProfilerEventForwarder, AZ::SystemAllocator);

        TracyProfilerEventForwarder() = default;
        ~TracyProfilerEventForwarder() = default;

        //! Registers/un-registers the AZ::Debug::Profiler instance to the interface
        void Init();
        void Shutdown();

        //! AZ::Debug::Profiler overrides...
        void BeginRegion(const AZ::Debug::Budget* budget, const char* eventName, ...) final override;
        void EndRegion(const AZ::Debug::Budget* budget) final override;

        //! AZ::TickBus::Handler overrides
        int GetTickOrder() override;
        void OnTick(float deltaTime, AZ::ScriptTimePoint timePoint) override;

    private:
        using EventIdStack = AZStd::vector<TracyCZoneCtx>;
        static thread_local EventIdStack ms_threadLocalStorage;

        // This lock will only be contested when the TracyProfilerEventForwarder's Shutdown() method has been called
        AZStd::shared_mutex m_shutdownMutex;

        bool m_initialized = false;
    };
} // namespace TracyProfiler
