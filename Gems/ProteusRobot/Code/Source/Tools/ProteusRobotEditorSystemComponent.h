
#pragma once

#include <AzToolsFramework/API/ToolsApplicationAPI.h>

#include <Clients/ProteusRobotSystemComponent.h>

namespace ProteusRobot
{
    /// System component for ProteusRobot editor
    class ProteusRobotEditorSystemComponent
        : public ProteusRobotSystemComponent
        , protected AzToolsFramework::EditorEvents::Bus::Handler
    {
        using BaseSystemComponent = ProteusRobotSystemComponent;
    public:
        AZ_COMPONENT(ProteusRobotEditorSystemComponent, "{1AD25C4B-B8F7-4C54-BF46-3F5A9E02E90B}", BaseSystemComponent);
        static void Reflect(AZ::ReflectContext* context);

        ProteusRobotEditorSystemComponent();
        ~ProteusRobotEditorSystemComponent();

    private:
        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);

        // AZ::Component
        void Activate() override;
        void Deactivate() override;
    };
} // namespace ProteusRobot
