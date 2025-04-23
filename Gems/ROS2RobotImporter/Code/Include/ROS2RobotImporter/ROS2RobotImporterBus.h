
#pragma once

#include <ROS2RobotImporter/ROS2RobotImporterTypeIds.h>

#include <AzCore/EBus/EBus.h>
#include <AzCore/Interface/Interface.h>

namespace ROS2RobotImporter
{
    class ROS2RobotImporterRequests
    {
    public:
        AZ_RTTI(ROS2RobotImporterRequests, ROS2RobotImporterRequestsTypeId);
        virtual ~ROS2RobotImporterRequests() = default;
        // Put your public methods here
    };

    class ROS2RobotImporterBusTraits
        : public AZ::EBusTraits
    {
    public:
        //////////////////////////////////////////////////////////////////////////
        // EBusTraits overrides
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::Single;
        //////////////////////////////////////////////////////////////////////////
    };

    using ROS2RobotImporterRequestBus = AZ::EBus<ROS2RobotImporterRequests, ROS2RobotImporterBusTraits>;
    using ROS2RobotImporterInterface = AZ::Interface<ROS2RobotImporterRequests>;

} // namespace ROS2RobotImporter
