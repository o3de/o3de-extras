
#pragma once

#include <AzToolsFramework/API/ToolsApplicationAPI.h>

#include <Clients/WarehouseAssetsSystemComponent.h>

namespace WarehouseAssets
{
    /// System component for WarehouseAssets editor
    class WarehouseAssetsEditorSystemComponent
        : public WarehouseAssetsSystemComponent
        , protected AzToolsFramework::EditorEvents::Bus::Handler
    {
        using BaseSystemComponent = WarehouseAssetsSystemComponent;
    public:
        AZ_COMPONENT(WarehouseAssetsEditorSystemComponent, "{595672C9-AD5F-4FB5-97C2-9891303B12F2}", BaseSystemComponent);
        static void Reflect(AZ::ReflectContext* context);

        WarehouseAssetsEditorSystemComponent();
        ~WarehouseAssetsEditorSystemComponent();

    private:
        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);

        // AZ::Component
        void Activate() override;
        void Deactivate() override;
    };
} // namespace WarehouseAssets
