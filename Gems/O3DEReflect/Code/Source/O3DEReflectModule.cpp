/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 */

#include <AzCore/Memory/SystemAllocator.h>
#include <AzCore/Module/Module.h>

#include "O3DEReflectSystemComponent.h"

namespace O3DEReflect
{
    class O3DEReflectModule
        : public AZ::Module
    {
    public:
        AZ_RTTI(O3DEReflectModule, "{A1B2C3D4-E5F6-7890-ABCD-EF1234567890}", AZ::Module);
        AZ_CLASS_ALLOCATOR(O3DEReflectModule, AZ::SystemAllocator);

        O3DEReflectModule()
            : AZ::Module()
        {
            m_descriptors.insert(m_descriptors.end(), {
                O3DEReflectSystemComponent::CreateDescriptor(),
            });
        }

        AZ::ComponentTypeList GetRequiredSystemComponents() const override
        {
            return AZ::ComponentTypeList{
                azrtti_typeid<O3DEReflectSystemComponent>(),
            };
        }
    };

} // namespace O3DEReflect

#if defined(O3DE_GEM_NAME)
AZ_DECLARE_MODULE_CLASS(AZ_JOIN(Gem_, O3DE_GEM_NAME), O3DEReflect::O3DEReflectModule)
#else
AZ_DECLARE_MODULE_CLASS(Gem_O3DEReflect, O3DEReflect::O3DEReflectModule)
#endif
