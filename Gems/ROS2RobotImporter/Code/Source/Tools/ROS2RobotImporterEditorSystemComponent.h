
#pragma once

#include <AzToolsFramework/API/ToolsApplicationAPI.h>

#include <Clients/ROS2RobotImporterSystemComponent.h>

namespace ROS2RobotImporter
{
    /// System component for ROS2RobotImporter editor
    class ROS2RobotImporterEditorSystemComponent
        : public ROS2RobotImporterSystemComponent
        , protected AzToolsFramework::EditorEvents::Bus::Handler
    {
        using BaseSystemComponent = ROS2RobotImporterSystemComponent;
    public:
        AZ_COMPONENT_DECL(ROS2RobotImporterEditorSystemComponent);

        static void Reflect(AZ::ReflectContext* context);

        ROS2RobotImporterEditorSystemComponent();
        ~ROS2RobotImporterEditorSystemComponent();

    private:
        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);

        // AZ::Component
        void Activate() override;
        void Deactivate() override;
    };
} // namespace ROS2RobotImporter
