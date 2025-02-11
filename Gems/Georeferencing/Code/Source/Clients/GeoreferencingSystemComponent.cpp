
#include "GeoreferencingSystemComponent.h"

#include <Georeferencing/GeoreferencingTypeIds.h>

#include <AzCore/Serialization/SerializeContext.h>

namespace Georeferencing
{
    AZ_COMPONENT_IMPL(GeoreferencingSystemComponent, "GeoreferencingSystemComponent",
        GeoreferencingSystemComponentTypeId);

    void GeoreferencingSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<GeoreferencingSystemComponent, AZ::Component>()
                ->Version(0)
                ;
        }
    }

    void GeoreferencingSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("GeoreferencingService"));
    }

    void GeoreferencingSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("GeoreferencingService"));
    }

    void GeoreferencingSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
    }

    void GeoreferencingSystemComponent::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
    }



    void GeoreferencingSystemComponent::Init()
    {
    }

    void GeoreferencingSystemComponent::Activate()
    {
    }

    void GeoreferencingSystemComponent::Deactivate()
    {

    }

} // namespace Georeferencing
