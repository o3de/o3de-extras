

#include <WarehouseAssetsModuleInterface.h>
#include "WarehouseAssetsSystemComponent.h"

namespace WarehouseAssets
{
    class WarehouseAssetsModule
        : public WarehouseAssetsModuleInterface
    {
    public:
        AZ_RTTI(WarehouseAssetsModule, "{D37D1896-0F04-46BB-89BC-B2A135DD8F64}", WarehouseAssetsModuleInterface);
        AZ_CLASS_ALLOCATOR(WarehouseAssetsModule, AZ::SystemAllocator, 0);
    };
}// namespace WarehouseAssets

AZ_DECLARE_MODULE_CLASS(Gem_WarehouseAssets, WarehouseAssets::WarehouseAssetsModule)
