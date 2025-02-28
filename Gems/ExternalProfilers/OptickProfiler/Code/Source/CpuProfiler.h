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
#include <AzCore/std/parallel/mutex.h>
#include <AzCore/std/parallel/shared_mutex.h>

namespace OptickProfiler
{
    class CpuProfiler final
        : public AZ::Debug::Profiler
        , public AZ::SystemTickBus::Handler
    {
    public:
        AZ_RTTI(CpuProfiler, "{E4076EA4-EF44-499A-9750-37B623BBBF7C}", AZ::Debug::Profiler);
        AZ_CLASS_ALLOCATOR(CpuProfiler, AZ::SystemAllocator);

        CpuProfiler() = default;
        ~CpuProfiler() = default;

        //! Registers/un-registers the AZ::Debug::Profiler instance to the interface
        void Init();
        void Shutdown();

        //! AZ::Debug::Profiler overrides...
        void BeginRegion(const AZ::Debug::Budget* budget, const char* eventName, ...) final override;
        void EndRegion(const AZ::Debug::Budget* budget) final override;

        //! AZ::SystemTickBus::Handler overrides
        void OnSystemTick() final override;

    private:
        // This lock will only be contested when the CpuProfiler's Shutdown() method has been called
        AZStd::shared_mutex m_shutdownMutex;

        bool m_initialized = false;
    };
} // namespace OptickProfiler
