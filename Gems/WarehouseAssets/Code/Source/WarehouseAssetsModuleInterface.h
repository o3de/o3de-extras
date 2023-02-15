
#include <AzCore/Memory/SystemAllocator.h>
#include <AzCore/Module/Module.h>
#include <Clients/WarehouseAssetsSystemComponent.h>

namespace WarehouseAssets
{
    class WarehouseAssetsModuleInterface
        : public AZ::Module
    {
    public:
        AZ_RTTI(WarehouseAssetsModuleInterface, "{FCA8F074-D249-413E-A4E3-EA038988624F}", AZ::Module);
        AZ_CLASS_ALLOCATOR(WarehouseAssetsModuleInterface, AZ::SystemAllocator, 0);

        WarehouseAssetsModuleInterface()
        {
            // Push results of [MyComponent]::CreateDescriptor() into m_descriptors here.
            // Add ALL components descriptors associated with this gem to m_descriptors.
            // This will associate the AzTypeInfo information for the components with the the SerializeContext, BehaviorContext and EditContext.
            // This happens through the [MyComponent]::Reflect() function.
            m_descriptors.insert(m_descriptors.end(), {
                WarehouseAssetsSystemComponent::CreateDescriptor(),
                });
        }

        /**
         * Add required SystemComponents to the SystemEntity.
         */
        AZ::ComponentTypeList GetRequiredSystemComponents() const override
        {
            return AZ::ComponentTypeList{
                azrtti_typeid<WarehouseAssetsSystemComponent>(),
            };
        }
    };
}// namespace WarehouseAssets
