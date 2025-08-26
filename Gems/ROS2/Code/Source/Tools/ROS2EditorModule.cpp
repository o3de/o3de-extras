/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ROS2EditorClockSystemComponent.h"
#include "ROS2EditorSystemComponent.h"
#include <Frame/ROS2FrameEditorComponent.h>
#include <Frame/ROS2FrameEditorSystemComponent.h>
#include <ROS2/ROS2TypeIds.h>
#include <ROS2ModuleInterface.h>
#ifdef WITH_GAZEBO_MSGS
#include <Spawner/ROS2SpawnPointEditorComponent.h>
#include <Spawner/ROS2SpawnerEditorComponent.h>
#endif

namespace ROS2
{
    class ROS2EditorModule : public ROS2ModuleInterface
    {
    public:
        AZ_RTTI(ROS2EditorModule, ROS2EditorModuleTypeId, ROS2ModuleInterface);
        AZ_CLASS_ALLOCATOR(ROS2EditorModule, AZ::SystemAllocator);

        ROS2EditorModule()
        {
            m_descriptors.insert(
                m_descriptors.end(),
                { ROS2EditorSystemComponent::CreateDescriptor(),
                  ROS2EditorClockSystemComponent::CreateDescriptor(),
#ifdef WITH_GAZEBO_MSGS
                  ROS2SpawnerEditorComponent::CreateDescriptor(),
                  ROS2SpawnPointEditorComponent::CreateDescriptor(),
#endif
                  ROS2FrameEditorSystemComponent::CreateDescriptor(),
                  ROS2FrameEditorComponent::CreateDescriptor() });
        }

        AZ::ComponentTypeList GetRequiredSystemComponents() const override
        {
            return AZ::ComponentTypeList{
                azrtti_typeid<ROS2EditorSystemComponent>(),
                azrtti_typeid<ROS2FrameEditorSystemComponent>(),
                azrtti_typeid<ROS2EditorClockSystemComponent>(),
            };
        }
    };
} // namespace ROS2

#if defined(O3DE_GEM_NAME)
AZ_DECLARE_MODULE_CLASS(AZ_JOIN(Gem_, O3DE_GEM_NAME, _Editor), ROS2::ROS2EditorModule)
#else
AZ_DECLARE_MODULE_CLASS(Gem_ROS2_Editor, ROS2::ROS2EditorModule)
#endif
