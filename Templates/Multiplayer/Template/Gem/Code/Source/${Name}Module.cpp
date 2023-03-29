/*
 * Copyright (c) Contributors to the Open 3D Engine Project. For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <AzCore/Memory/SystemAllocator.h>
#include <AzCore/Module/Module.h>
#include <Components/ExampleFilteredEntityComponent.h>
#include <Components//NetworkPrefabSpawnerComponent.h>
#include <Source/AutoGen/AutoComponentTypes.h>

#include "${SanitizedCppName}SystemComponent.h"

namespace ${SanitizedCppName}
{
    class ${SanitizedCppName}Module
        : public AZ::Module
    {
    public:
        AZ_RTTI(${SanitizedCppName}Module, "${ModuleClassId}", AZ::Module);
        AZ_CLASS_ALLOCATOR(${SanitizedCppName}Module, AZ::SystemAllocator);

        ${SanitizedCppName}Module()
            : AZ::Module()
        {
            // Push results of [MyComponent]::CreateDescriptor() into m_descriptors here.
            m_descriptors.insert(m_descriptors.end(), {
                ${SanitizedCppName}SystemComponent::CreateDescriptor(),
                ExampleFilteredEntityComponent::CreateDescriptor(),
                NetworkPrefabSpawnerComponent::CreateDescriptor(),
            });

            CreateComponentDescriptors(m_descriptors);
        }

        /**
         * Add required SystemComponents to the SystemEntity.
         */
        AZ::ComponentTypeList GetRequiredSystemComponents() const override
        {
            return AZ::ComponentTypeList{
                azrtti_typeid<${SanitizedCppName}SystemComponent>(),
            };
        }
    };
}

// DO NOT MODIFY THIS LINE UNLESS YOU RENAME THE GEM
// The first parameter should be GemName_GemIdLower
// The second should be the fully qualified name of the class above
#if defined(AZ_MONOLITHIC_BUILD)
AZ_DECLARE_MODULE_CLASS(Gem_${SanitizedCppName}_Client, ${SanitizedCppName}::${SanitizedCppName}Module);
AZ_DECLARE_MODULE_CLASS(Gem_${SanitizedCppName}_Server, ${SanitizedCppName}::${SanitizedCppName}Module);
#endif
AZ_DECLARE_MODULE_CLASS(Gem_${SanitizedCppName}, ${SanitizedCppName}::${SanitizedCppName}Module)
