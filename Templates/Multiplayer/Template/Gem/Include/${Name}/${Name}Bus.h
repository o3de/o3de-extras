
#pragma once

#include <${SanitizedCppName}/${SanitizedCppName}TypeIds.h>

#include <AzCore/EBus/EBus.h>
#include <AzCore/Interface/Interface.h>

namespace ${SanitizedCppName}
{
    class ${SanitizedCppName}Requests
    {
    public:
        AZ_RTTI(${SanitizedCppName}Requests, ${SanitizedCppName}RequestsTypeId);
        virtual ~${SanitizedCppName}Requests() = default;
        // Put your public methods here
    };

    class ${SanitizedCppName}BusTraits
        : public AZ::EBusTraits
    {
    public:
        //////////////////////////////////////////////////////////////////////////
        // EBusTraits overrides
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::Single;
        //////////////////////////////////////////////////////////////////////////
    };

    using ${SanitizedCppName}RequestBus = AZ::EBus<${SanitizedCppName}Requests, ${SanitizedCppName}BusTraits>;
    using ${SanitizedCppName}Interface = AZ::Interface<${SanitizedCppName}Requests>;

} // namespace ${SanitizedCppName}
