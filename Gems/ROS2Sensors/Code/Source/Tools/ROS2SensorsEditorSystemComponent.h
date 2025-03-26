
#pragma once

#include <AzToolsFramework/API/ToolsApplicationAPI.h>

#include <Clients/ROS2SensorsSystemComponent.h>

namespace ROS2Sensors
{
    /// System component for ROS2Sensors editor
    class ROS2SensorsEditorSystemComponent
        : public ROS2SensorsSystemComponent
        , protected AzToolsFramework::EditorEvents::Bus::Handler
    {
        using BaseSystemComponent = ROS2SensorsSystemComponent;

    public:
        AZ_COMPONENT_DECL(ROS2SensorsEditorSystemComponent);

        static void Reflect(AZ::ReflectContext* context);

        ROS2SensorsEditorSystemComponent();
        ~ROS2SensorsEditorSystemComponent();

    private:
        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);

        // AZ::Component
        void Activate() override;
        void Deactivate() override;
    };
} // namespace ROS2Sensors
