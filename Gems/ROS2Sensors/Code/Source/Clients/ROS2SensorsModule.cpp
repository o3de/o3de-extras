
#include "ROS2SensorsSystemComponent.h"
#include <ROS2Sensors/ROS2SensorsTypeIds.h>
#include <ROS2SensorsModuleInterface.h>

namespace ROS2Sensors
{
    class ROS2SensorsModule : public ROS2SensorsModuleInterface
    {
    public:
        AZ_RTTI(ROS2SensorsModule, ROS2SensorsModuleTypeId, ROS2SensorsModuleInterface);
        AZ_CLASS_ALLOCATOR(ROS2SensorsModule, AZ::SystemAllocator);
    };
} // namespace ROS2Sensors

#if defined(O3DE_GEM_NAME)
AZ_DECLARE_MODULE_CLASS(AZ_JOIN(Gem_, O3DE_GEM_NAME), ROS2Sensors::ROS2SensorsModule)
#else
AZ_DECLARE_MODULE_CLASS(Gem_ROS2Sensors, ROS2Sensors::ROS2SensorsModule)
#endif
