
#pragma once

#include <AzToolsFramework/API/ToolsApplicationAPI.h>

#include <MachineLearningSystemComponent.h>

namespace MachineLearning
{
    /// System component for MachineLearning editor
    class MachineLearningEditorSystemComponent
        : public MachineLearningSystemComponent
        , protected AzToolsFramework::EditorEvents::Bus::Handler
    {
        using BaseSystemComponent = MachineLearningSystemComponent;
    public:
        AZ_COMPONENT_DECL(MachineLearningEditorSystemComponent);

        static void Reflect(AZ::ReflectContext* context);

        MachineLearningEditorSystemComponent();
        ~MachineLearningEditorSystemComponent();

    private:
        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);

        // AZ::Component
        void Activate() override;
        void Deactivate() override;
    };
} // namespace MachineLearning
