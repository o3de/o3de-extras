
#include <AzCore/Memory/SystemAllocator.h>
#include <AzCore/Module/Module.h>
#include <Clients/ProteusRobotSystemComponent.h>

namespace ProteusRobot
{
    class ProteusRobotModuleInterface
        : public AZ::Module
    {
    public:
        AZ_RTTI(ProteusRobotModuleInterface, "{C2853683-8867-42E4-BAF8-6BAE29CB53E1}", AZ::Module);
        AZ_CLASS_ALLOCATOR(ProteusRobotModuleInterface, AZ::SystemAllocator, 0);

        ProteusRobotModuleInterface()
        {
            // Push results of [MyComponent]::CreateDescriptor() into m_descriptors here.
            // Add ALL components descriptors associated with this gem to m_descriptors.
            // This will associate the AzTypeInfo information for the components with the the SerializeContext, BehaviorContext and EditContext.
            // This happens through the [MyComponent]::Reflect() function.
            m_descriptors.insert(m_descriptors.end(), {
                ProteusRobotSystemComponent::CreateDescriptor(),
                });
        }

        /**
         * Add required SystemComponents to the SystemEntity.
         */
        AZ::ComponentTypeList GetRequiredSystemComponents() const override
        {
            return AZ::ComponentTypeList{
                azrtti_typeid<ProteusRobotSystemComponent>(),
            };
        }
    };
}// namespace ProteusRobot
