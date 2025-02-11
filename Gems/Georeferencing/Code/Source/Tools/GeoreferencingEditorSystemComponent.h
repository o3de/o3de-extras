
#pragma once

#include <AzToolsFramework/API/ToolsApplicationAPI.h>

#include <Clients/GeoreferencingSystemComponent.h>

namespace Georeferencing
{
    /// System component for Georeferencing editor
    class GeoreferencingEditorSystemComponent
        : public GeoreferencingSystemComponent
        , protected AzToolsFramework::EditorEvents::Bus::Handler
    {
        using BaseSystemComponent = GeoreferencingSystemComponent;
    public:
        AZ_COMPONENT_DECL(GeoreferencingEditorSystemComponent);

        static void Reflect(AZ::ReflectContext* context);

        GeoreferencingEditorSystemComponent();
        ~GeoreferencingEditorSystemComponent();

    private:
        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);

        // AZ::Component
        void Activate() override;
        void Deactivate() override;
    };
} // namespace Georeferencing
