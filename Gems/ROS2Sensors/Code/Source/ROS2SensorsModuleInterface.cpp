/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ROS2SensorsModuleInterface.h"
#include <AzCore/Memory/Memory.h>

#include <ROS2Sensors/ROS2SensorsTypeIds.h>

#include <Clients/ROS2SensorsSystemComponent.h>

namespace ROS2Sensors
{
    AZ_TYPE_INFO_WITH_NAME_IMPL(ROS2SensorsModuleInterface, "ROS2SensorsModuleInterface", ROS2SensorsModuleInterfaceTypeId);
    AZ_RTTI_NO_TYPE_INFO_IMPL(ROS2SensorsModuleInterface, AZ::Module);
    AZ_CLASS_ALLOCATOR_IMPL(ROS2SensorsModuleInterface, AZ::SystemAllocator);

    ROS2SensorsModuleInterface::ROS2SensorsModuleInterface()
    {
        // Push results of [MyComponent]::CreateDescriptor() into m_descriptors here.
        // Add ALL components descriptors associated with this gem to m_descriptors.
        // This will associate the AzTypeInfo information for the components with the the SerializeContext, BehaviorContext and EditContext.
        // This happens through the [MyComponent]::Reflect() function.
        m_descriptors.insert(
            m_descriptors.end(),
            {
                ROS2SensorsSystemComponent::CreateDescriptor(),
            });
    }

    AZ::ComponentTypeList ROS2SensorsModuleInterface::GetRequiredSystemComponents() const
    {
        return AZ::ComponentTypeList{
            azrtti_typeid<ROS2SensorsSystemComponent>(),
        };
    }
} // namespace ROS2Sensors
