/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include "Frame/ROS2FrameComponent.h"
#include "Imu/ROS2ImuSensorComponent.h"
#include "Lidar/ROS2LidarSensorComponent.h"
#include "RobotControl/ROS2RobotControlComponent.h"
#include "ROS2SystemComponent.h"
#include <AzCore/Memory/SystemAllocator.h>
#include <AzCore/Module/Module.h>

namespace ROS2
{
    class ROS2ModuleInterface
        : public AZ::Module
    {
    public:
        AZ_RTTI(ROS2ModuleInterface, "{8b5567cb-1de9-49af-9cd4-9750d4abcd6b}", AZ::Module);
        AZ_CLASS_ALLOCATOR(ROS2ModuleInterface, AZ::SystemAllocator, 0);

        ROS2ModuleInterface()
        {
            // Push results of [MyComponent]::CreateDescriptor() into m_descriptors here.
            // Add ALL components descriptors associated with this gem to m_descriptors.
            // This will associate the AzTypeInfo information for the components with the SerializeContext, BehaviorContext and EditContext.
            // This happens through the [MyComponent]::Reflect() function.
            m_descriptors.insert(m_descriptors.end(), {
                ROS2SystemComponent::CreateDescriptor(),
                ROS2SensorComponent::CreateDescriptor(),
                ROS2ImuSensorComponent::CreateDescriptor(),
                ROS2LidarSensorComponent::CreateDescriptor(),
                ROS2FrameComponent::CreateDescriptor(),
                ROS2RobotControlComponent::CreateDescriptor()
                });
        }

        //! Add required SystemComponents to the SystemEntity.
        AZ::ComponentTypeList GetRequiredSystemComponents() const override
        {
            return AZ::ComponentTypeList{
                azrtti_typeid<ROS2SystemComponent>(),
            };
        }
    };
}  // namespace ROS2
