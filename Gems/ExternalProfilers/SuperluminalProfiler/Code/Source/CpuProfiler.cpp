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
#include <Superluminal/PerformanceAPI.h>

namespace SuperluminalProfiler
{
	void CpuProfiler::Init()
	{
		AZ::Interface<AZ::Debug::Profiler>::Register(this);
		m_initialized = true;
		AZ::SystemTickBus::Handler::BusConnect();
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
	}

	void CpuProfiler::BeginRegion(const AZ::Debug::Budget* budget, const char* eventName, ...)
	{
		// Try to lock here, the shutdownMutex will only be contested when the CpuProfiler is shutting down.
		if (m_shutdownMutex.try_lock_shared())
		{
			PerformanceAPI_BeginEvent(eventName, budget->Name(), budget->Crc());
			m_shutdownMutex.unlock_shared();
		}
	}

	void CpuProfiler::EndRegion([[maybe_unused]] const AZ::Debug::Budget* budget)
	{
		// Try to lock here, the shutdownMutex will only be contested when the CpuProfiler is shutting down.
		if (m_shutdownMutex.try_lock_shared())
		{
			PerformanceAPI_EndEvent();
			m_shutdownMutex.unlock_shared();
		}
	}

	void CpuProfiler::OnSystemTick()
	{
		PerformanceAPI::InstrumentationScope("SystemTick", nullptr, PERFORMANCEAPI_MAKE_COLOR(0, 255, 0));
	}

}
