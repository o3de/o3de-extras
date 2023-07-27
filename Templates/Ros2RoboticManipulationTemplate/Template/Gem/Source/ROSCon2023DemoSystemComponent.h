
#pragma once

#include <AzCore/Component/Component.h>

#include <${Name}/${Name}Bus.h>

namespace ${Name}
{
    class ${Name}SystemComponent
        : public AZ::Component
        , protected ${Name}RequestBus::Handler
    {
    public:
        AZ_COMPONENT(${Name}SystemComponent, "{${Random_Uuid}}");

        static void Reflect(AZ::ReflectContext* context);

        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);

        ${Name}SystemComponent();
        ~${Name}SystemComponent();

    protected:
        ////////////////////////////////////////////////////////////////////////
        // ${Name}RequestBus interface implementation

        ////////////////////////////////////////////////////////////////////////

        ////////////////////////////////////////////////////////////////////////
        // AZ::Component interface implementation
        void Init() override;
        void Activate() override;
        void Deactivate() override;
        ////////////////////////////////////////////////////////////////////////
    };
}
