
#include <AzCore/Serialization/SerializeContext.h>
#include "ROS2SensorsEditorSystemComponent.h"

#include <ROS2Sensors/ROS2SensorsTypeIds.h>

namespace ROS2Sensors
{
    AZ_COMPONENT_IMPL(ROS2SensorsEditorSystemComponent, "ROS2SensorsEditorSystemComponent",
        ROS2SensorsEditorSystemComponentTypeId, BaseSystemComponent);

    void ROS2SensorsEditorSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<ROS2SensorsEditorSystemComponent, ROS2SensorsSystemComponent>()
                ->Version(0);
        }
    }

    ROS2SensorsEditorSystemComponent::ROS2SensorsEditorSystemComponent() = default;

    ROS2SensorsEditorSystemComponent::~ROS2SensorsEditorSystemComponent() = default;

    void ROS2SensorsEditorSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        BaseSystemComponent::GetProvidedServices(provided);
        provided.push_back(AZ_CRC_CE("ROS2SensorsEditorService"));
    }

    void ROS2SensorsEditorSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        BaseSystemComponent::GetIncompatibleServices(incompatible);
        incompatible.push_back(AZ_CRC_CE("ROS2SensorsEditorService"));
    }

    void ROS2SensorsEditorSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        BaseSystemComponent::GetRequiredServices(required);
    }

    void ROS2SensorsEditorSystemComponent::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
        BaseSystemComponent::GetDependentServices(dependent);
    }

    void ROS2SensorsEditorSystemComponent::Activate()
    {
        ROS2SensorsSystemComponent::Activate();
        AzToolsFramework::EditorEvents::Bus::Handler::BusConnect();
    }

    void ROS2SensorsEditorSystemComponent::Deactivate()
    {
        AzToolsFramework::EditorEvents::Bus::Handler::BusDisconnect();
        ROS2SensorsSystemComponent::Deactivate();
    }

} // namespace ROS2Sensors
