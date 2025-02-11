
#pragma once

#include <AzCore/Component/Component.h>
#include <AzCore/Component/TickBus.h>

namespace Georeferencing
{
    class GeoreferencingSystemComponent
        : public AZ::Component
    {
    public:
        AZ_COMPONENT_DECL(GeoreferencingSystemComponent);

        static void Reflect(AZ::ReflectContext* context);

        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);

        GeoreferencingSystemComponent() = default;
        ~GeoreferencingSystemComponent() = default;

    protected:

        // AZ::Component interface implementation
        void Init() override;
        void Activate() override;
        void Deactivate() override;

    };

} // namespace Georeferencing
