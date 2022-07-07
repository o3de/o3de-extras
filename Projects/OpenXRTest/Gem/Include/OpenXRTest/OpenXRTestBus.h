
#pragma once

#include <AzCore/EBus/EBus.h>
#include <AzCore/Interface/Interface.h>

namespace OpenXRTest
{
    class OpenXRTestRequests
    {
    public:
        AZ_RTTI(OpenXRTestRequests, "{ee9807e7-e5f4-4669-ac2e-800d7d034158}");
        virtual ~OpenXRTestRequests() = default;
        // Put your public methods here
    };

    class OpenXRTestBusTraits
        : public AZ::EBusTraits
    {
    public:
        //////////////////////////////////////////////////////////////////////////
        // EBusTraits overrides
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::Single;
        //////////////////////////////////////////////////////////////////////////
    };

    using OpenXRTestRequestBus = AZ::EBus<OpenXRTestRequests, OpenXRTestBusTraits>;
    using OpenXRTestInterface = AZ::Interface<OpenXRTestRequests>;

} // namespace OpenXRTest
