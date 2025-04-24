
#include "ROS2ModuleInterface.h"
#include <AzCore/Memory/Memory.h>

#include <ROS2/ROS2TypeIds.h>

#include <Clients/ROS2SystemComponent.h>

namespace ROS2
{
    AZ_TYPE_INFO_WITH_NAME_IMPL(ROS2ModuleInterface, "ROS2ModuleInterface", ROS2ModuleInterfaceTypeId);
    AZ_RTTI_NO_TYPE_INFO_IMPL(ROS2ModuleInterface, AZ::Module);
    AZ_CLASS_ALLOCATOR_IMPL(ROS2ModuleInterface, AZ::SystemAllocator);

    ROS2ModuleInterface::ROS2ModuleInterface()
    {
        // Push results of [MyComponent]::CreateDescriptor() into m_descriptors here.
        // Add ALL components descriptors associated with this gem to m_descriptors.
        // This will associate the AzTypeInfo information for the components with the the SerializeContext, BehaviorContext and EditContext.
        // This happens through the [MyComponent]::Reflect() function.
        m_descriptors.insert(
            m_descriptors.end(),
            {
                ROS2SystemComponent::CreateDescriptor(),
            });
    }

    AZ::ComponentTypeList ROS2ModuleInterface::GetRequiredSystemComponents() const
    {
        return AZ::ComponentTypeList{
            azrtti_typeid<ROS2SystemComponent>(),
        };
    }
} // namespace ROS2
