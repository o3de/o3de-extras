/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <ProfilerSystemComponent.h>

#include <AzCore/Memory/SystemAllocator.h>
#include <AzCore/Module/Module.h>

namespace SuperluminalProfiler
{
	class ProfilerModule
		: public AZ::Module
	{
	public:
		AZ_RTTI(ProfilerModule, "{7C666AFB-E699-42FB-8F32-DAEE5A62CD01}", AZ::Module);
		AZ_CLASS_ALLOCATOR(ProfilerModule, AZ::SystemAllocator);

		ProfilerModule()
		{
			// Push results of [MyComponent]::CreateDescriptor() into m_descriptors here.
			// Add ALL components descriptors associated with this gem to m_descriptors.
			// This will associate the AzTypeInfo information for the components with the the SerializeContext, BehaviorContext and EditContext.
			// This happens through the [MyComponent]::Reflect() function.
			m_descriptors.insert(m_descriptors.end(), {
				ProfilerSystemComponent::CreateDescriptor(),
			});
		}

		/**
		 * Add required SystemComponents to the SystemEntity.
		 */
		AZ::ComponentTypeList GetRequiredSystemComponents() const override
		{
			return AZ::ComponentTypeList{
				azrtti_typeid<ProfilerSystemComponent>(),
			};
		}
	};
}

#if defined(O3DE_GEM_NAME)
AZ_DECLARE_MODULE_CLASS(AZ_JOIN(Gem_, O3DE_GEM_NAME), SuperluminalProfiler::ProfilerModule)
#else
AZ_DECLARE_MODULE_CLASS(Gem_Profiler, SuperluminalProfiler::ProfilerModule)
#endif
