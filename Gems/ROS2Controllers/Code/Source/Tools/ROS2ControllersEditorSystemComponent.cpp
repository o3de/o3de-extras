
#include "ROS2ControllersEditorSystemComponent.h"
#include <AzCore/Serialization/SerializeContext.h>

#include <ROS2Controllers/ROS2ControllersTypeIds.h>

namespace ROS2Controllers {
AZ_COMPONENT_IMPL(ROS2ControllersEditorSystemComponent,
                  "ROS2ControllersEditorSystemComponent",
                  ROS2ControllersEditorSystemComponentTypeId,
                  BaseSystemComponent);

void ROS2ControllersEditorSystemComponent::Reflect(
    AZ::ReflectContext *context) {
  if (auto serializeContext = azrtti_cast<AZ::SerializeContext *>(context)) {
    serializeContext
        ->Class<ROS2ControllersEditorSystemComponent,
                ROS2ControllersSystemComponent>()
        ->Version(0);
  }
}

ROS2ControllersEditorSystemComponent::ROS2ControllersEditorSystemComponent() =
    default;

ROS2ControllersEditorSystemComponent::~ROS2ControllersEditorSystemComponent() =
    default;

void ROS2ControllersEditorSystemComponent::GetProvidedServices(
    AZ::ComponentDescriptor::DependencyArrayType &provided) {
  BaseSystemComponent::GetProvidedServices(provided);
  provided.push_back(AZ_CRC_CE("ROS2ControllersEditorService"));
}

void ROS2ControllersEditorSystemComponent::GetIncompatibleServices(
    AZ::ComponentDescriptor::DependencyArrayType &incompatible) {
  BaseSystemComponent::GetIncompatibleServices(incompatible);
  incompatible.push_back(AZ_CRC_CE("ROS2ControllersEditorService"));
}

void ROS2ControllersEditorSystemComponent::GetRequiredServices(
    [[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType &required) {
  BaseSystemComponent::GetRequiredServices(required);
}

void ROS2ControllersEditorSystemComponent::GetDependentServices(
    [[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType &dependent) {
  BaseSystemComponent::GetDependentServices(dependent);
}

void ROS2ControllersEditorSystemComponent::Activate() {
  ROS2ControllersSystemComponent::Activate();
  AzToolsFramework::EditorEvents::Bus::Handler::BusConnect();
}

void ROS2ControllersEditorSystemComponent::Deactivate() {
  AzToolsFramework::EditorEvents::Bus::Handler::BusDisconnect();
  ROS2ControllersSystemComponent::Deactivate();
}

} // namespace ROS2Controllers
