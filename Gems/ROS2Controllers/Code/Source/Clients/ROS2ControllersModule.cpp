
#include <ROS2Controllers/ROS2ControllersTypeIds.h>
#include <ROS2ControllersModuleInterface.h>
#include "ROS2ControllersSystemComponent.h"

namespace ROS2Controllers
{
    class ROS2ControllersModule
        : public ROS2ControllersModuleInterface
    {
    public:
        AZ_RTTI(ROS2ControllersModule, ROS2ControllersModuleTypeId, ROS2ControllersModuleInterface);
        AZ_CLASS_ALLOCATOR(ROS2ControllersModule, AZ::SystemAllocator);
    };
}// namespace ROS2Controllers

#if defined(O3DE_GEM_NAME)
AZ_DECLARE_MODULE_CLASS(AZ_JOIN(Gem_, O3DE_GEM_NAME), ROS2Controllers::ROS2ControllersModule)
#else
AZ_DECLARE_MODULE_CLASS(Gem_ROS2Controllers, ROS2Controllers::ROS2ControllersModule)
#endif
