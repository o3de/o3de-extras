
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>

#include "OpenXRTestSystemComponent.h"

namespace OpenXRTest
{
    void OpenXRTestSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<OpenXRTestSystemComponent, AZ::Component>()
                ->Version(0)
                ;

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<OpenXRTestSystemComponent>("OpenXRTest", "[Description of functionality provided by this System Component]")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("System"))
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ;
            }
        }
    }

    void OpenXRTestSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC("OpenXRTestService"));
    }

    void OpenXRTestSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC("OpenXRTestService"));
    }

    void OpenXRTestSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
    }

    void OpenXRTestSystemComponent::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
    }

    OpenXRTestSystemComponent::OpenXRTestSystemComponent()
    {
        if (OpenXRTestInterface::Get() == nullptr)
        {
            OpenXRTestInterface::Register(this);
        }
    }

    OpenXRTestSystemComponent::~OpenXRTestSystemComponent()
    {
        if (OpenXRTestInterface::Get() == this)
        {
            OpenXRTestInterface::Unregister(this);
        }
    }

    void OpenXRTestSystemComponent::Init()
    {
    }

    void OpenXRTestSystemComponent::Activate()
    {
        OpenXRTestRequestBus::Handler::BusConnect();
    }

    void OpenXRTestSystemComponent::Deactivate()
    {
        OpenXRTestRequestBus::Handler::BusDisconnect();
    }
}
