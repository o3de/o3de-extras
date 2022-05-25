/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <AzCore/Memory/SystemAllocator.h>
#include <AzCore/Module/Module.h>
#include <OpenXrVk/OpenXrVkSystemComponent.h>

namespace OpenXRVk
{   
    //! This module is in charge of loading system components related to Openxrvk. 
    class Module
        : public AZ::Module
    {
    public:
        AZ_RTTI(Module, "{C34AA64E-0983-4D30-A33C-0D7C7676A20E}", AZ::Module);
        AZ_CLASS_ALLOCATOR(Module, AZ::SystemAllocator, 0);

        Module()
            : AZ::Module()
        {
            m_descriptors.insert(m_descriptors.end(), {
                    SystemComponent::CreateDescriptor(),
            });
        }

        AZ::ComponentTypeList GetRequiredSystemComponents() const override
        {
            return
            {
                azrtti_typeid<OpenXRVk::SystemComponent>()
            };
        }
    };
}


// DO NOT MODIFY THIS LINE UNLESS YOU RENAME THE GEM
// The first parameter should be GemName_GemIdLower
// The second should be the fully qualified name of the class above
AZ_DECLARE_MODULE_CLASS(Gem_OpenXRVk, OpenXRVk::Module)
