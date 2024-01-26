/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <AzCore/Memory/SystemAllocator.h>
#include <AzCore/Module/Module.h>

#include "OpenXRAssetBuildersSystemComponent.h"

namespace OpenXRVkBuilders
{
    class OpenXRBuilderModule final
        : public AZ::Module
    {
    public:
        AZ_RTTI(OpenXRBuilderModule, "{43370465-DBF1-44BB-968D-97C0B42F5EA0}", AZ::Module);
        AZ_CLASS_ALLOCATOR(OpenXRBuilderModule, AZ::SystemAllocator);
    
        OpenXRBuilderModule()
        {
            // Push results of [MyComponent]::CreateDescriptor() into m_descriptors here.
            m_descriptors.insert(m_descriptors.end(), {
                OpenXRAssetBuildersSystemComponent::CreateDescriptor(),
            });
        }
    
        /**
         * Add required SystemComponents to the SystemEntity.
         */
        AZ::ComponentTypeList GetRequiredSystemComponents() const override
        {
            return AZ::ComponentTypeList{
                azrtti_typeid<OpenXRAssetBuildersSystemComponent>(),
            };
        }
    };
} // namespace OpenXRVkBuilders

#if defined(O3DE_GEM_NAME)
AZ_DECLARE_MODULE_CLASS(AZ_JOIN(Gem_, O3DE_GEM_NAME, _Builders), OpenXRVkBuilders::OpenXRBuilderModule)
#else
AZ_DECLARE_MODULE_CLASS(Gem_OpenXRVk_Builders, OpenXRVkBuilders::OpenXRBuilderModule)
#endif
