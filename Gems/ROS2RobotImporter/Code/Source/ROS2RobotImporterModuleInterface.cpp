
#include "ROS2RobotImporterModuleInterface.h"
#include <AzCore/Memory/Memory.h>

#include <ROS2RobotImporter/ROS2RobotImporterTypeIds.h>

#include <Clients/ROS2RobotImporterSystemComponent.h>

namespace ROS2RobotImporter
{
    AZ_TYPE_INFO_WITH_NAME_IMPL(ROS2RobotImporterModuleInterface,
        "ROS2RobotImporterModuleInterface", ROS2RobotImporterModuleInterfaceTypeId);
    AZ_RTTI_NO_TYPE_INFO_IMPL(ROS2RobotImporterModuleInterface, AZ::Module);
    AZ_CLASS_ALLOCATOR_IMPL(ROS2RobotImporterModuleInterface, AZ::SystemAllocator);

    ROS2RobotImporterModuleInterface::ROS2RobotImporterModuleInterface()
    {
        // Push results of [MyComponent]::CreateDescriptor() into m_descriptors here.
        // Add ALL components descriptors associated with this gem to m_descriptors.
        // This will associate the AzTypeInfo information for the components with the the SerializeContext, BehaviorContext and EditContext.
        // This happens through the [MyComponent]::Reflect() function.
        m_descriptors.insert(m_descriptors.end(), {
            ROS2RobotImporterSystemComponent::CreateDescriptor(),
            });
    }

    AZ::ComponentTypeList ROS2RobotImporterModuleInterface::GetRequiredSystemComponents() const
    {
        return AZ::ComponentTypeList{
            azrtti_typeid<ROS2RobotImporterSystemComponent>(),
        };
    }
} // namespace ROS2RobotImporter
