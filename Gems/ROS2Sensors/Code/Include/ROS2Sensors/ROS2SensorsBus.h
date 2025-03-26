
#pragma once

#include <ROS2Sensors/ROS2SensorsTypeIds.h>

#include <AzCore/EBus/EBus.h>
#include <AzCore/Interface/Interface.h>

namespace ROS2Sensors
{
    class ROS2SensorsRequests
    {
    public:
        AZ_RTTI(ROS2SensorsRequests, ROS2SensorsRequestsTypeId);
        virtual ~ROS2SensorsRequests() = default;
        // Put your public methods here
    };

    class ROS2SensorsBusTraits
        : public AZ::EBusTraits
    {
    public:
        //////////////////////////////////////////////////////////////////////////
        // EBusTraits overrides
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::Single;
        //////////////////////////////////////////////////////////////////////////
    };

    using ROS2SensorsRequestBus = AZ::EBus<ROS2SensorsRequests, ROS2SensorsBusTraits>;
    using ROS2SensorsInterface = AZ::Interface<ROS2SensorsRequests>;

} // namespace ROS2Sensors
