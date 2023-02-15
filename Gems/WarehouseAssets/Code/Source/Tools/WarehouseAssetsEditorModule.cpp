
#include <WarehouseAssetsModuleInterface.h>
#include "WarehouseAssetsEditorSystemComponent.h"

namespace WarehouseAssets
{
    class WarehouseAssetsEditorModule
        : public WarehouseAssetsModuleInterface
    {
    public:
        AZ_RTTI(WarehouseAssetsEditorModule, "{D37D1896-0F04-46BB-89BC-B2A135DD8F64}", WarehouseAssetsModuleInterface);
        AZ_CLASS_ALLOCATOR(WarehouseAssetsEditorModule, AZ::SystemAllocator, 0);

        WarehouseAssetsEditorModule()
        {
            // Push results of [MyComponent]::CreateDescriptor() into m_descriptors here.
            // Add ALL components descriptors associated with this gem to m_descriptors.
            // This will associate the AzTypeInfo information for the components with the the SerializeContext, BehaviorContext and EditContext.
            // This happens through the [MyComponent]::Reflect() function.
            m_descriptors.insert(m_descriptors.end(), {
                WarehouseAssetsEditorSystemComponent::CreateDescriptor(),
            });
        }

        /**
         * Add required SystemComponents to the SystemEntity.
         * Non-SystemComponents should not be added here
         */
        AZ::ComponentTypeList GetRequiredSystemComponents() const override
        {
            return AZ::ComponentTypeList {
                azrtti_typeid<WarehouseAssetsEditorSystemComponent>(),
            };
        }
    };
}// namespace WarehouseAssets

AZ_DECLARE_MODULE_CLASS(Gem_WarehouseAssets, WarehouseAssets::WarehouseAssetsEditorModule)
