
#include <AzCore/Serialization/SerializeContext.h>
#include "MachineLearningEditorSystemComponent.h"

#include <MachineLearning/MachineLearningTypeIds.h>

namespace MachineLearning
{
    AZ_COMPONENT_IMPL(MachineLearningEditorSystemComponent, "MachineLearningEditorSystemComponent",
        MachineLearningEditorSystemComponentTypeId, BaseSystemComponent);

    void MachineLearningEditorSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<MachineLearningEditorSystemComponent, MachineLearningSystemComponent>()
                ->Version(0);
        }
    }

    MachineLearningEditorSystemComponent::MachineLearningEditorSystemComponent() = default;

    MachineLearningEditorSystemComponent::~MachineLearningEditorSystemComponent() = default;

    void MachineLearningEditorSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        BaseSystemComponent::GetProvidedServices(provided);
        provided.push_back(AZ_CRC_CE("MachineLearningEditorService"));
    }

    void MachineLearningEditorSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        BaseSystemComponent::GetIncompatibleServices(incompatible);
        incompatible.push_back(AZ_CRC_CE("MachineLearningEditorService"));
    }

    void MachineLearningEditorSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        BaseSystemComponent::GetRequiredServices(required);
    }

    void MachineLearningEditorSystemComponent::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
        BaseSystemComponent::GetDependentServices(dependent);
    }

    void MachineLearningEditorSystemComponent::Activate()
    {
        MachineLearningSystemComponent::Activate();
        AzToolsFramework::EditorEvents::Bus::Handler::BusConnect();
    }

    void MachineLearningEditorSystemComponent::Deactivate()
    {
        AzToolsFramework::EditorEvents::Bus::Handler::BusDisconnect();
        MachineLearningSystemComponent::Deactivate();
    }

} // namespace MachineLearning
