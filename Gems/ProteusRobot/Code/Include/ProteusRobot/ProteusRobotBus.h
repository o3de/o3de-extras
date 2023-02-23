
#pragma once

#include <AzCore/EBus/EBus.h>
#include <AzCore/Interface/Interface.h>

namespace ProteusRobot
{
    class ProteusRobotRequests
    {
    public:
        AZ_RTTI(ProteusRobotRequests, "{2480DA88-89C0-40D0-9C27-CA19C99FECC7}");
        virtual ~ProteusRobotRequests() = default;
        // Put your public methods here
    };
    
    class ProteusRobotBusTraits
        : public AZ::EBusTraits
    {
    public:
        //////////////////////////////////////////////////////////////////////////
        // EBusTraits overrides
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::Single;
        //////////////////////////////////////////////////////////////////////////
    };

    using ProteusRobotRequestBus = AZ::EBus<ProteusRobotRequests, ProteusRobotBusTraits>;
    using ProteusRobotInterface = AZ::Interface<ProteusRobotRequests>;

} // namespace ProteusRobot
