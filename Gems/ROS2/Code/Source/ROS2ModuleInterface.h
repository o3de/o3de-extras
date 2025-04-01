/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Memory/SystemAllocator.h>
#include <AzCore/Module/Module.h>
#include <ROS2/Frame/ROS2FrameComponent.h>
#include <RobotImporter/ROS2RobotImporterSystemComponent.h>
#include <SimulationUtils/FollowingCameraComponent.h>
#include <Spawner/ROS2SpawnPointComponent.h>
#include <Spawner/ROS2SpawnerComponent.h>
#include <SystemComponents/ROS2SystemComponent.h>

namespace ROS2
{
    class ROS2ModuleInterface : public AZ::Module
    {
    public:
        AZ_RTTI(ROS2ModuleInterface, "{8b5567cb-1de9-49af-9cd4-9750d4abcd6b}", AZ::Module);
        AZ_CLASS_ALLOCATOR(ROS2ModuleInterface, AZ::SystemAllocator);

        ROS2ModuleInterface()
        {
            m_descriptors.insert(
                m_descriptors.end(),
                {
                    ROS2SystemComponent::CreateDescriptor(),
                    ROS2RobotImporterSystemComponent::CreateDescriptor(),
                    ROS2FrameComponent::CreateDescriptor(),
                    ROS2SpawnerComponent::CreateDescriptor(),
                    ROS2SpawnPointComponent::CreateDescriptor(),
                    FollowingCameraComponent::CreateDescriptor(),
                });
        }

        AZ::ComponentTypeList GetRequiredSystemComponents() const override
        {
            return AZ::ComponentTypeList{
                azrtti_typeid<ROS2SystemComponent>(),
                azrtti_typeid<ROS2RobotImporterSystemComponent>(),
            };
        }
    };
} // namespace ROS2
