
#include <ProteusRobotModuleInterface.h>
#include "ProteusRobotEditorSystemComponent.h"

namespace ProteusRobot
{
    class ProteusRobotEditorModule
        : public ProteusRobotModuleInterface
    {
    public:
        AZ_RTTI(ProteusRobotEditorModule, "{F9558D3E-566B-4824-8634-015F21864F5E}", ProteusRobotModuleInterface);
        AZ_CLASS_ALLOCATOR(ProteusRobotEditorModule, AZ::SystemAllocator);

        ProteusRobotEditorModule()
        {
            // Push results of [MyComponent]::CreateDescriptor() into m_descriptors here.
            // Add ALL components descriptors associated with this gem to m_descriptors.
            // This will associate the AzTypeInfo information for the components with the the SerializeContext, BehaviorContext and EditContext.
            // This happens through the [MyComponent]::Reflect() function.
            m_descriptors.insert(m_descriptors.end(), {
                ProteusRobotEditorSystemComponent::CreateDescriptor(),
            });
        }

        /**
         * Add required SystemComponents to the SystemEntity.
         * Non-SystemComponents should not be added here
         */
        AZ::ComponentTypeList GetRequiredSystemComponents() const override
        {
            return AZ::ComponentTypeList {
                azrtti_typeid<ProteusRobotEditorSystemComponent>(),
            };
        }
    };
}// namespace ProteusRobot

AZ_DECLARE_MODULE_CLASS(Gem_ProteusRobot, ProteusRobot::ProteusRobotEditorModule)
