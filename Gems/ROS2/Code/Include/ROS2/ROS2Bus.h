
#pragma once

#include <ROS2/ROS2TypeIds.h>

#include <AzCore/EBus/EBus.h>
#include <AzCore/Interface/Interface.h>

namespace ROS2
{
    class ROS2Requests
    {
    public:
        AZ_RTTI(ROS2Requests, ROS2RequestsTypeId);
        virtual ~ROS2Requests() = default;
        // Put your public methods here
    };

    class ROS2BusTraits : public AZ::EBusTraits
    {
    public:
        //////////////////////////////////////////////////////////////////////////
        // EBusTraits overrides
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::Single;
        //////////////////////////////////////////////////////////////////////////
    };

    using ROS2RequestBus = AZ::EBus<ROS2Requests, ROS2BusTraits>;
    using ROS2Interface = AZ::Interface<ROS2Requests>;

} // namespace ROS2
