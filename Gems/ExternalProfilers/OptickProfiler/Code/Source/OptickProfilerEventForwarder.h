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
#include <AzCore/std/containers/unordered_map.h>
#include <AzCore/std/parallel/mutex.h>
#include <AzCore/std/parallel/shared_mutex.h>
#include <AzCore/std/parallel/threadbus.h>

namespace Optick
{
    struct EventDescription;
    struct EventStorage;
} // namespace Optick

namespace OptickProfiler
{
    //! Listen to 03DE frame/profiling events and forward them to the Optick profiling library
    class OptickProfilerEventForwarder final
        : public AZ::Debug::Profiler
        , public AZ::TickBus::Handler
        , public AZStd::ThreadEventBus::Handler
    {
    public:
        AZ_RTTI(OptickProfilerEventForwarder, "{E4076EA4-EF44-499A-9750-37B623BBBF7C}", AZ::Debug::Profiler);
        AZ_CLASS_ALLOCATOR(OptickProfilerEventForwarder, AZ::SystemAllocator);

        OptickProfilerEventForwarder() = default;
        ~OptickProfilerEventForwarder() = default;

        //! Registers/un-registers the AZ::Debug::Profiler instance to the interface
        void Init();
        void Shutdown();

        //! AZ::Debug::Profiler overrides
        void BeginRegion(const AZ::Debug::Budget* budget, const char* eventName, ...) final override;
        void EndRegion(const AZ::Debug::Budget* budget) final override;

        //! AZ::TickBus::Handler overrides
        int GetTickOrder() final override;
        void OnTick(float deltaTime, AZ::ScriptTimePoint timePoint) final override;

        //! AZStd::ThreadEventBus::Handler overrides
        virtual void OnThreadEnter(const AZStd::thread::id& id, const AZStd::thread_desc* desc) final override;
        virtual void OnThreadExit(const AZStd::thread::id& id) final override;

    private:
        // This lock will only be contested when the OptickProfilerEventForwarder's Shutdown() method has been called
        AZStd::shared_mutex m_shutdownMutex;

        AZStd::shared_mutex m_threadIdToNameMutex;
        AZStd::unordered_map<AZStd::native_thread_id_type, AZStd::string> m_threadIdToName;

        static thread_local Optick::EventStorage* m_pOptickStorage;
        Optick::EventDescription* m_pOptickFrameTag = nullptr;

        bool m_initialized = false;
    };
} // namespace OptickProfiler
