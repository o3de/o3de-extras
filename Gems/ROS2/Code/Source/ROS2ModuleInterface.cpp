/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ROS2ModuleInterface.h"
#include <AzCore/Memory/Memory.h>

#include <Clients/ROS2SystemComponent.h>
#include <ROS2/Frame/ROS2FrameComponent.h>
#include <ROS2/ROS2TypeIds.h>
#include <ROS2/Sensor/Events/PhysicsBasedSource.h>
#include <ROS2/Sensor/Events/TickBasedSource.h>
#include <ROS2/Sensor/ROS2SensorComponentBase.h>
#include <SimulationUtils/FollowingCameraComponent.h>
#include <Spawner/ROS2SpawnPointComponent.h>
#include <Spawner/ROS2SpawnerComponent.h>

namespace ROS2
{
    AZ_TYPE_INFO_WITH_NAME_IMPL(ROS2ModuleInterface, "ROS2ModuleInterface", ROS2ModuleInterfaceTypeId);
    AZ_RTTI_NO_TYPE_INFO_IMPL(ROS2ModuleInterface, AZ::Module);
    AZ_CLASS_ALLOCATOR_IMPL(ROS2ModuleInterface, AZ::SystemAllocator);

    ROS2ModuleInterface::ROS2ModuleInterface()
    {
        m_descriptors.insert(
            m_descriptors.end(),
            {
                ROS2SystemComponent::CreateDescriptor(),
                ROS2FrameComponent::CreateDescriptor(),
                ROS2SpawnerComponent::CreateDescriptor(),
                ROS2SpawnPointComponent::CreateDescriptor(),
                FollowingCameraComponent::CreateDescriptor(),
                ROS2SensorComponentBase<TickBasedSource>::CreateDescriptor(),
                ROS2SensorComponentBase<PhysicsBasedSource>::CreateDescriptor(),
            });
    }

    AZ::ComponentTypeList ROS2ModuleInterface::GetRequiredSystemComponents() const
    {
        return AZ::ComponentTypeList{
            azrtti_typeid<ROS2SystemComponent>(),
        };
    }
} // namespace ROS2
