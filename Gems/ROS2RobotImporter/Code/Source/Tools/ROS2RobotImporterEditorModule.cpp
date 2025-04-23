
#include "ROS2RobotImporterEditorSystemComponent.h"
#include <ROS2RobotImporter/ROS2RobotImporterTypeIds.h>
#include <ROS2RobotImporterModuleInterface.h>

namespace ROS2RobotImporter
{
    class ROS2RobotImporterEditorModule : public ROS2RobotImporterModuleInterface
    {
    public:
        AZ_RTTI(ROS2RobotImporterEditorModule, ROS2RobotImporterEditorModuleTypeId, ROS2RobotImporterModuleInterface);
        AZ_CLASS_ALLOCATOR(ROS2RobotImporterEditorModule, AZ::SystemAllocator);

        ROS2RobotImporterEditorModule()
        {
            // Push results of [MyComponent]::CreateDescriptor() into m_descriptors
            // here. Add ALL components descriptors associated with this gem to
            // m_descriptors. This will associate the AzTypeInfo information for the
            // components with the the SerializeContext, BehaviorContext and
            // EditContext. This happens through the [MyComponent]::Reflect() function.
            m_descriptors.insert(
                m_descriptors.end(),
                {
                    ROS2RobotImporterEditorSystemComponent::CreateDescriptor(),
                });
        }

        /**
         * Add required SystemComponents to the SystemEntity.
         * Non-SystemComponents should not be added here
         */
        AZ::ComponentTypeList GetRequiredSystemComponents() const override
        {
            return AZ::ComponentTypeList{
                azrtti_typeid<ROS2RobotImporterEditorSystemComponent>(),
            };
        }
    };
} // namespace ROS2RobotImporter

#if defined(O3DE_GEM_NAME)
AZ_DECLARE_MODULE_CLASS(AZ_JOIN(Gem_, O3DE_GEM_NAME, _Editor), ROS2RobotImporter::ROS2RobotImporterEditorModule)
#else
AZ_DECLARE_MODULE_CLASS(Gem_ROS2RobotImporter_Editor, ROS2RobotImporter::ROS2RobotImporterEditorModule)
#endif
