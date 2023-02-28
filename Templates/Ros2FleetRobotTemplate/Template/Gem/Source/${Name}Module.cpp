// {BEGIN_LICENSE}
/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
// {END_LICENSE}

#include <AzCore/Memory/SystemAllocator.h>
#include <AzCore/Module/Module.h>

#include "${Name}SystemComponent.h"

namespace ${Name}
{
    class ${Name}Module
        : public AZ::Module
    {
    public:
        AZ_RTTI(${Name}Module, "{${Random_Uuid}}", AZ::Module);
        AZ_CLASS_ALLOCATOR(${Name}Module, AZ::SystemAllocator, 0);

        ${Name}Module()
            : AZ::Module()
        {
            // Push results of [MyComponent]::CreateDescriptor() into m_descriptors here.
            m_descriptors.insert(m_descriptors.end(), {
                ${Name}SystemComponent::CreateDescriptor(),
            });
        }

        /**
         * Add required SystemComponents to the SystemEntity.
         */
        AZ::ComponentTypeList GetRequiredSystemComponents() const override
        {
            return AZ::ComponentTypeList{
                azrtti_typeid<${Name}SystemComponent>(),
            };
        }
    };
}// namespace ${Name}

AZ_DECLARE_MODULE_CLASS(Gem_${Name}, ${Name}::${Name}Module)
