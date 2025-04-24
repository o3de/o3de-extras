
#include "ROS2SystemComponent.h"
#include <ROS2/ROS2TypeIds.h>
#include <ROS2ModuleInterface.h>

namespace ROS2
{
    class ROS2Module : public ROS2ModuleInterface
    {
    public:
        AZ_RTTI(ROS2Module, ROS2ModuleTypeId, ROS2ModuleInterface);
        AZ_CLASS_ALLOCATOR(ROS2Module, AZ::SystemAllocator);
    };
} // namespace ROS2

#if defined(O3DE_GEM_NAME)
AZ_DECLARE_MODULE_CLASS(AZ_JOIN(Gem_, O3DE_GEM_NAME), ROS2::ROS2Module)
#else
AZ_DECLARE_MODULE_CLASS(Gem_ROS2, ROS2::ROS2Module)
#endif
