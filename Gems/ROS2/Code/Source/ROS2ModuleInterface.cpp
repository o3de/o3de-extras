
#include "ROS2ModuleInterface.h"
#include <AzCore/Memory/Memory.h>

#include <ROS2/ROS2TypeIds.h>

#include <Clients/ROS2SystemComponent.h>
#include <ROS2/Frame/ROS2FrameComponent.h>
#include <SimulationUtils/FollowingCameraComponent.h>
#include <Spawner/ROS2SpawnPointComponent.h>
#include <Spawner/ROS2SpawnerComponent.h>

namespace ROS2
{
    AZ_TYPE_INFO_WITH_NAME_IMPL(ROS2ModuleInterface, "ROS2ModuleInterface", ROS2ModuleInterfaceTypeId);
    AZ_RTTI_NO_TYPE_INFO_IMPL(ROS2ModuleInterface, AZ::Module);
    AZ_CLASS_ALLOCATOR_IMPL(ROS2ModuleInterface, AZ::SystemAllocator);

    ROS2ModuleInterface::ROS2ModuleInterface()
    {
        m_descriptors.insert(
            m_descriptors.end(),
            {
                ROS2SystemComponent::CreateDescriptor(),
                ROS2FrameComponent::CreateDescriptor(),
                ROS2SpawnerComponent::CreateDescriptor(),
                ROS2SpawnPointComponent::CreateDescriptor(),
                FollowingCameraComponent::CreateDescriptor(),
            });
    }

    AZ::ComponentTypeList ROS2ModuleInterface::GetRequiredSystemComponents() const
    {
        return AZ::ComponentTypeList{
            azrtti_typeid<ROS2SystemComponent>(),
        };
    }
} // namespace ROS2
