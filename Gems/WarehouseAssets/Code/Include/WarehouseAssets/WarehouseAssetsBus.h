
#pragma once

#include <AzCore/EBus/EBus.h>
#include <AzCore/Interface/Interface.h>

namespace WarehouseAssets
{
    class WarehouseAssetsRequests
    {
    public:
        AZ_RTTI(WarehouseAssetsRequests, "{6DFE3CA5-3F22-48CA-A800-A002AC4E0626}");
        virtual ~WarehouseAssetsRequests() = default;
        // Put your public methods here
    };
    
    class WarehouseAssetsBusTraits
        : public AZ::EBusTraits
    {
    public:
        //////////////////////////////////////////////////////////////////////////
        // EBusTraits overrides
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::Single;
        //////////////////////////////////////////////////////////////////////////
    };

    using WarehouseAssetsRequestBus = AZ::EBus<WarehouseAssetsRequests, WarehouseAssetsBusTraits>;
    using WarehouseAssetsInterface = AZ::Interface<WarehouseAssetsRequests>;

} // namespace WarehouseAssets
