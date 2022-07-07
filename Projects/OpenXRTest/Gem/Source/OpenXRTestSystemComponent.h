
#pragma once

#include <AzCore/Component/Component.h>

#include <OpenXRTest/OpenXRTestBus.h>

namespace OpenXRTest
{
    class OpenXRTestSystemComponent
        : public AZ::Component
        , protected OpenXRTestRequestBus::Handler
    {
    public:
        AZ_COMPONENT(OpenXRTestSystemComponent, "{6625841a-3d71-4752-839e-83a14b5a8b73}");

        static void Reflect(AZ::ReflectContext* context);

        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);

        OpenXRTestSystemComponent();
        ~OpenXRTestSystemComponent();

    protected:
        ////////////////////////////////////////////////////////////////////////
        // OpenXRTestRequestBus interface implementation

        ////////////////////////////////////////////////////////////////////////

        ////////////////////////////////////////////////////////////////////////
        // AZ::Component interface implementation
        void Init() override;
        void Activate() override;
        void Deactivate() override;
        ////////////////////////////////////////////////////////////////////////
    };
}
