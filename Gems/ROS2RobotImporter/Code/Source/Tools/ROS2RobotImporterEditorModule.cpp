
#include "ROS2RobotImporterEditorSystemComponent.h"
#include <QtCore/qglobal.h>
#include <ROS2RobotImporter/ROS2RobotImporterTypeIds.h>
#include <ROS2RobotImporterModuleInterface.h>
#include <SdfAssetBuilder/SdfAssetBuilderSystemComponent.h>

void InitQrcResources()
{
    // Registration of Qt (ROS2RobotImporter.qrc) resources
    Q_INIT_RESOURCE(ROS2RobotImporter);
}

namespace ROS2RobotImporter
{
    class ROS2RobotImporterEditorModule : public ROS2RobotImporterModuleInterface
    {
    public:
        AZ_RTTI(ROS2RobotImporterEditorModule, ROS2RobotImporterEditorModuleTypeId, ROS2RobotImporterModuleInterface);
        AZ_CLASS_ALLOCATOR(ROS2RobotImporterEditorModule, AZ::SystemAllocator);

        ROS2RobotImporterEditorModule()
        {
            InitQrcResources();

            m_descriptors.insert(
                m_descriptors.end(),
                {
                    ROS2::ROS2RobotImporterEditorSystemComponent::CreateDescriptor(),
                    ROS2::SdfAssetBuilderSystemComponent::CreateDescriptor(),
                });
        }

        AZ::ComponentTypeList GetRequiredSystemComponents() const override
        {
            return AZ::ComponentTypeList{
                azrtti_typeid<ROS2::ROS2RobotImporterEditorSystemComponent>(),
                azrtti_typeid<ROS2::SdfAssetBuilderSystemComponent>(),
            };
        }
    };
} // namespace ROS2RobotImporter

#if defined(O3DE_GEM_NAME)
AZ_DECLARE_MODULE_CLASS(AZ_JOIN(Gem_, O3DE_GEM_NAME, _Editor), ROS2RobotImporter::ROS2RobotImporterEditorModule)
#else
AZ_DECLARE_MODULE_CLASS(Gem_ROS2RobotImporter_Editor, ROS2RobotImporter::ROS2RobotImporterEditorModule)
#endif
