
#include <AzCore/Memory/SystemAllocator.h>
#include <AzCore/Module/Module.h>

#include "${Name}SystemComponent.h"

#include <${SanitizedCppName}/${SanitizedCppName}TypeIds.h>
#include <Components/AttachPlayerWeaponComponent.h>
#include <Source/AutoGen/AutoComponentTypes.h>


namespace ${SanitizedCppName}
{
    class ${SanitizedCppName}Module
        : public AZ::Module
    {
    public:
        AZ_RTTI(${SanitizedCppName}Module, ${SanitizedCppName}ModuleTypeId, AZ::Module);
        AZ_CLASS_ALLOCATOR(${SanitizedCppName}Module, AZ::SystemAllocator);

        ${SanitizedCppName}Module()
            : AZ::Module()
        {
            // Push results of [MyComponent]::CreateDescriptor() into m_descriptors here.
            m_descriptors.insert(m_descriptors.end(), {
                ${SanitizedCppName}SystemComponent::CreateDescriptor(),
                AttachPlayerWeaponComponent::CreateDescriptor(),
            });

            // Register multiplayer components
            CreateComponentDescriptors(m_descriptors);
        }

        /**
         * Add required SystemComponents to the SystemEntity.
         */
        AZ::ComponentTypeList GetRequiredSystemComponents() const override
        {
            return AZ::ComponentTypeList{
                azrtti_typeid<${SanitizedCppName}SystemComponent>(),
            };
        }
    };
}// namespace ${SanitizedCppName}

#if defined(O3DE_GEM_NAME)
AZ_DECLARE_MODULE_CLASS(AZ_JOIN(Gem_, O3DE_GEM_NAME), ${SanitizedCppName}::${SanitizedCppName}Module)
#else
AZ_DECLARE_MODULE_CLASS(Gem_${SanitizedCppName}, ${SanitizedCppName}::${SanitizedCppName}Module)
#endif
