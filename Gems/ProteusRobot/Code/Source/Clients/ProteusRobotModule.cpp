

#include <ProteusRobotModuleInterface.h>
#include "ProteusRobotSystemComponent.h"

namespace ProteusRobot
{
    class ProteusRobotModule
        : public ProteusRobotModuleInterface
    {
    public:
        AZ_RTTI(ProteusRobotModule, "{F9558D3E-566B-4824-8634-015F21864F5E}", ProteusRobotModuleInterface);
        AZ_CLASS_ALLOCATOR(ProteusRobotModule, AZ::SystemAllocator);
    };
}// namespace ProteusRobot

AZ_DECLARE_MODULE_CLASS(Gem_ProteusRobot, ProteusRobot::ProteusRobotModule)
