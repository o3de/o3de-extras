
#include <AzCore/Serialization/SerializeContext.h>
#include "ProteusRobotEditorSystemComponent.h"

namespace ProteusRobot
{
    void ProteusRobotEditorSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<ProteusRobotEditorSystemComponent, ProteusRobotSystemComponent>()
                ->Version(0);
        }
    }

    ProteusRobotEditorSystemComponent::ProteusRobotEditorSystemComponent() = default;

    ProteusRobotEditorSystemComponent::~ProteusRobotEditorSystemComponent() = default;

    void ProteusRobotEditorSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        BaseSystemComponent::GetProvidedServices(provided);
        provided.push_back(AZ_CRC_CE("ProteusRobotEditorService"));
    }

    void ProteusRobotEditorSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        BaseSystemComponent::GetIncompatibleServices(incompatible);
        incompatible.push_back(AZ_CRC_CE("ProteusRobotEditorService"));
    }

    void ProteusRobotEditorSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        BaseSystemComponent::GetRequiredServices(required);
    }

    void ProteusRobotEditorSystemComponent::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
        BaseSystemComponent::GetDependentServices(dependent);
    }

    void ProteusRobotEditorSystemComponent::Activate()
    {
        ProteusRobotSystemComponent::Activate();
        AzToolsFramework::EditorEvents::Bus::Handler::BusConnect();
    }

    void ProteusRobotEditorSystemComponent::Deactivate()
    {
        AzToolsFramework::EditorEvents::Bus::Handler::BusDisconnect();
        ProteusRobotSystemComponent::Deactivate();
    }

} // namespace ProteusRobot
