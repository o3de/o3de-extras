
#include "WarehouseAssetsSystemComponent.h"

#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>

namespace WarehouseAssets
{
    void WarehouseAssetsSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<WarehouseAssetsSystemComponent, AZ::Component>()
                ->Version(0)
                ;

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<WarehouseAssetsSystemComponent>("WarehouseAssets", "[Description of functionality provided by this System Component]")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("System"))
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ;
            }
        }
    }

    void WarehouseAssetsSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("WarehouseAssetsService"));
    }

    void WarehouseAssetsSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("WarehouseAssetsService"));
    }

    void WarehouseAssetsSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
    }

    void WarehouseAssetsSystemComponent::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
    }

    WarehouseAssetsSystemComponent::WarehouseAssetsSystemComponent()
    {
        if (WarehouseAssetsInterface::Get() == nullptr)
        {
            WarehouseAssetsInterface::Register(this);
        }
    }

    WarehouseAssetsSystemComponent::~WarehouseAssetsSystemComponent()
    {
        if (WarehouseAssetsInterface::Get() == this)
        {
            WarehouseAssetsInterface::Unregister(this);
        }
    }

    void WarehouseAssetsSystemComponent::Init()
    {
    }

    void WarehouseAssetsSystemComponent::Activate()
    {
        WarehouseAssetsRequestBus::Handler::BusConnect();
        AZ::TickBus::Handler::BusConnect();
    }

    void WarehouseAssetsSystemComponent::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
        WarehouseAssetsRequestBus::Handler::BusDisconnect();
    }

    void WarehouseAssetsSystemComponent::OnTick([[maybe_unused]] float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
    }

} // namespace WarehouseAssets
