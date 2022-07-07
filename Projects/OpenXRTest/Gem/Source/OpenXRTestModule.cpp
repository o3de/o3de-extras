
#include <AzCore/Memory/SystemAllocator.h>
#include <AzCore/Module/Module.h>

#include "OpenXRTestSystemComponent.h"

namespace OpenXRTest
{
    class OpenXRTestModule
        : public AZ::Module
    {
    public:
        AZ_RTTI(OpenXRTestModule, "{5f1ec07d-a7bb-4816-aa3d-abc2104071ae}", AZ::Module);
        AZ_CLASS_ALLOCATOR(OpenXRTestModule, AZ::SystemAllocator, 0);

        OpenXRTestModule()
            : AZ::Module()
        {
            // Push results of [MyComponent]::CreateDescriptor() into m_descriptors here.
            m_descriptors.insert(m_descriptors.end(), {
                OpenXRTestSystemComponent::CreateDescriptor(),
            });
        }

        /**
         * Add required SystemComponents to the SystemEntity.
         */
        AZ::ComponentTypeList GetRequiredSystemComponents() const override
        {
            return AZ::ComponentTypeList{
                azrtti_typeid<OpenXRTestSystemComponent>(),
            };
        }
    };
}// namespace OpenXRTest

AZ_DECLARE_MODULE_CLASS(Gem_OpenXRTest, OpenXRTest::OpenXRTestModule)
