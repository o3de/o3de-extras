
#include "ROS2ControllersEditorSystemComponent.h"
#include <ROS2Controllers/ROS2ControllersTypeIds.h>
#include <ROS2ControllersModuleInterface.h>

namespace ROS2Controllers {
class ROS2ControllersEditorModule : public ROS2ControllersModuleInterface {
public:
  AZ_RTTI(ROS2ControllersEditorModule, ROS2ControllersEditorModuleTypeId,
          ROS2ControllersModuleInterface);
  AZ_CLASS_ALLOCATOR(ROS2ControllersEditorModule, AZ::SystemAllocator);

  ROS2ControllersEditorModule() {
    // Push results of [MyComponent]::CreateDescriptor() into m_descriptors
    // here. Add ALL components descriptors associated with this gem to
    // m_descriptors. This will associate the AzTypeInfo information for the
    // components with the the SerializeContext, BehaviorContext and
    // EditContext. This happens through the [MyComponent]::Reflect() function.
    m_descriptors.insert(
        m_descriptors.end(),
        {
            ROS2ControllersEditorSystemComponent::CreateDescriptor(),
        });
  }

  /**
   * Add required SystemComponents to the SystemEntity.
   * Non-SystemComponents should not be added here
   */
  AZ::ComponentTypeList GetRequiredSystemComponents() const override {
    return AZ::ComponentTypeList{
        azrtti_typeid<ROS2ControllersEditorSystemComponent>(),
    };
  }
};
} // namespace ROS2Controllers

#if defined(O3DE_GEM_NAME)
AZ_DECLARE_MODULE_CLASS(AZ_JOIN(Gem_, O3DE_GEM_NAME, _Editor),
                        ROS2Controllers::ROS2ControllersEditorModule)
#else
AZ_DECLARE_MODULE_CLASS(Gem_ROS2Controllers_Editor,
                        ROS2Controllers::ROS2ControllersEditorModule)
#endif
