/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "GeoreferenceLevelEditorComponent.h"
#include "GeoreferencingEditorSystemComponent.h"
#include <Georeferencing/GeoreferencingTypeIds.h>
#include <GeoreferencingModuleInterface.h>
namespace Georeferencing
{
    class GeoreferencingEditorModule : public GeoreferencingModuleInterface
    {
    public:
        AZ_RTTI(GeoreferencingEditorModule, GeoreferencingEditorModuleTypeId, GeoreferencingModuleInterface);
        AZ_CLASS_ALLOCATOR(GeoreferencingEditorModule, AZ::SystemAllocator);

        GeoreferencingEditorModule()
        {
            // Push results of [MyComponent]::CreateDescriptor() into m_descriptors here.
            // Add ALL components descriptors associated with this gem to m_descriptors.
            // This will associate the AzTypeInfo information for the components with the the SerializeContext, BehaviorContext and
            // EditContext. This happens through the [MyComponent]::Reflect() function.
            m_descriptors.insert(
                m_descriptors.end(),
                {
                    GeoreferencingEditorSystemComponent::CreateDescriptor(),
                    GeoReferenceLevelEditorComponent::CreateDescriptor(),
                });
        }

        /**
         * Add required SystemComponents to the SystemEntity.
         * Non-SystemComponents should not be added here
         */
        AZ::ComponentTypeList GetRequiredSystemComponents() const override
        {
            return AZ::ComponentTypeList{
                azrtti_typeid<GeoreferencingEditorSystemComponent>(),
            };
        }
    };
} // namespace Georeferencing

#if defined(O3DE_GEM_NAME)
AZ_DECLARE_MODULE_CLASS(AZ_JOIN(Gem_, O3DE_GEM_NAME, _Editor), Georeferencing::GeoreferencingEditorModule)
#else
AZ_DECLARE_MODULE_CLASS(Gem_LevelGeoreferencing_Editor, Georeferencing::GeoreferencingEditorModule)
#endif
