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

namespace SuperluminalProfiler
{
    class CpuProfiler final : public AZ::Debug::Profiler
    {
    public:
        AZ_RTTI(CpuProfiler, "{A05E7DC4-AB00-41BC-A739-8E58908CB84F}", AZ::Debug::Profiler);
        AZ_CLASS_ALLOCATOR(CpuProfiler, AZ::SystemAllocator);

        CpuProfiler() = default;
        ~CpuProfiler() = default;

        //! Registers/un-registers the AZ::Debug::Profiler instance to the interface
        void Init();
        void Shutdown();

        //! AZ::Debug::Profiler overrides...
        void BeginRegion(const AZ::Debug::Budget* budget, const char* eventName, ...) final override;
        void EndRegion(const AZ::Debug::Budget* budget) final override;

        //! Check to see if a programmatic capture is currently in progress, implies
        //! that the profiler is active if returns True.
        bool IsContinuousCaptureInProgress() const;

    private:
        // This lock will only be contested when the CpuProfiler's Shutdown() method has been called
        AZStd::shared_mutex m_shutdownMutex;

        bool m_initialized = false;
    };
} // namespace SuperluminalProfiler
