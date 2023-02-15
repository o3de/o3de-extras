
#include <AzCore/Serialization/SerializeContext.h>
#include "WarehouseAssetsEditorSystemComponent.h"

namespace WarehouseAssets
{
    void WarehouseAssetsEditorSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<WarehouseAssetsEditorSystemComponent, WarehouseAssetsSystemComponent>()
                ->Version(0);
        }
    }

    WarehouseAssetsEditorSystemComponent::WarehouseAssetsEditorSystemComponent() = default;

    WarehouseAssetsEditorSystemComponent::~WarehouseAssetsEditorSystemComponent() = default;

    void WarehouseAssetsEditorSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        BaseSystemComponent::GetProvidedServices(provided);
        provided.push_back(AZ_CRC_CE("WarehouseAssetsEditorService"));
    }

    void WarehouseAssetsEditorSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        BaseSystemComponent::GetIncompatibleServices(incompatible);
        incompatible.push_back(AZ_CRC_CE("WarehouseAssetsEditorService"));
    }

    void WarehouseAssetsEditorSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        BaseSystemComponent::GetRequiredServices(required);
    }

    void WarehouseAssetsEditorSystemComponent::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
        BaseSystemComponent::GetDependentServices(dependent);
    }

    void WarehouseAssetsEditorSystemComponent::Activate()
    {
        WarehouseAssetsSystemComponent::Activate();
        AzToolsFramework::EditorEvents::Bus::Handler::BusConnect();
    }

    void WarehouseAssetsEditorSystemComponent::Deactivate()
    {
        AzToolsFramework::EditorEvents::Bus::Handler::BusDisconnect();
        WarehouseAssetsSystemComponent::Deactivate();
    }

} // namespace WarehouseAssets
