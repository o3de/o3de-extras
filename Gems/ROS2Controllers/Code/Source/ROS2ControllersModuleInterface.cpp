/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root
 * of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ROS2ControllersModuleInterface.h"
#include <AzCore/Memory/Memory.h>

#include <ROS2Controllers/ROS2ControllersTypeIds.h>

#include <Clients/ROS2ControllersSystemComponent.h>

namespace ROS2Controllers {
AZ_TYPE_INFO_WITH_NAME_IMPL(ROS2ControllersModuleInterface,
                            "ROS2ControllersModuleInterface",
                            ROS2ControllersModuleInterfaceTypeId);
AZ_RTTI_NO_TYPE_INFO_IMPL(ROS2ControllersModuleInterface, AZ::Module);
AZ_CLASS_ALLOCATOR_IMPL(ROS2ControllersModuleInterface, AZ::SystemAllocator);

ROS2ControllersModuleInterface::ROS2ControllersModuleInterface() {
  // Push results of [MyComponent]::CreateDescriptor() into m_descriptors here.
  // Add ALL components descriptors associated with this gem to m_descriptors.
  // This will associate the AzTypeInfo information for the components with the
  // the SerializeContext, BehaviorContext and EditContext. This happens through
  // the [MyComponent]::Reflect() function.
  m_descriptors.insert(m_descriptors.end(),
                       {
                           ROS2ControllersSystemComponent::CreateDescriptor(),
                       });
}

AZ::ComponentTypeList
ROS2ControllersModuleInterface::GetRequiredSystemComponents() const {
  return AZ::ComponentTypeList{
      azrtti_typeid<ROS2ControllersSystemComponent>(),
  };
}
} // namespace ROS2Controllers
