/*
* Copyright (c) Contributors to the Open 3D Engine Project.
* For complete copyright and license terms please see the LICENSE at the root of this distribution.
*
* SPDX-License-Identifier: Apache-2.0 OR MIT
*
*/

#include "GeoreferencingModuleInterface.h"
#include <AzCore/Memory/Memory.h>

#include <Georeferencing/GeoreferencingTypeIds.h>

#include <Clients/GeoreferenceLevelComponent.h>
#include <Clients/GeoreferencingSystemComponent.h>
namespace Georeferencing
{
    AZ_TYPE_INFO_WITH_NAME_IMPL(GeoreferencingModuleInterface, "GeoreferencingModuleInterface", GeoreferencingModuleInterfaceTypeId);
    AZ_RTTI_NO_TYPE_INFO_IMPL(GeoreferencingModuleInterface, AZ::Module);
    AZ_CLASS_ALLOCATOR_IMPL(GeoreferencingModuleInterface, AZ::SystemAllocator);

    GeoreferencingModuleInterface::GeoreferencingModuleInterface()
    {
        // Push results of [MyComponent]::CreateDescriptor() into m_descriptors here.
        // Add ALL components descriptors associated with this gem to m_descriptors.
        // This will associate the AzTypeInfo information for the components with the the SerializeContext, BehaviorContext and EditContext.
        // This happens through the [MyComponent]::Reflect() function.
        m_descriptors.insert(
            m_descriptors.end(),
            {
                GeoreferencingSystemComponent::CreateDescriptor(),
                GeoReferenceLevelComponent::CreateDescriptor(),
            });
    }

    AZ::ComponentTypeList GeoreferencingModuleInterface::GetRequiredSystemComponents() const
    {
        return AZ::ComponentTypeList{
            azrtti_typeid<GeoreferencingSystemComponent>(),
        };
    }
} // namespace Georeferencing
