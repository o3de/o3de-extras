/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ROS2SensorsEditorSystemComponent.h"
#include <ROS2Sensors/ROS2SensorsTypeIds.h>
#include <ROS2SensorsModuleInterface.h>

#include <Camera/ROS2CameraSensorEditorComponent.h>
#include <Camera/ROS2EditorCameraSystemComponent.h>
#include <Lidar/LidarRegistrarEditorSystemComponent.h>

namespace ROS2Sensors
{
    class ROS2SensorsEditorModule : public ROS2SensorsModuleInterface
    {
    public:
        AZ_RTTI(ROS2SensorsEditorModule, ROS2SensorsEditorModuleTypeId, ROS2SensorsModuleInterface);
        AZ_CLASS_ALLOCATOR(ROS2SensorsEditorModule, AZ::SystemAllocator);

        ROS2SensorsEditorModule()
        {
            m_descriptors.insert(
                m_descriptors.end(),
                {
                    ROS2SensorsEditorSystemComponent::CreateDescriptor(),
                    ROS2CameraSensorEditorComponent::CreateDescriptor(),
                    ROS2EditorCameraSystemComponent::CreateDescriptor(),
                    LidarRegistrarEditorSystemComponent::CreateDescriptor(),
                });
        }

        AZ::ComponentTypeList GetRequiredSystemComponents() const override
        {
            return AZ::ComponentTypeList{
                azrtti_typeid<ROS2SensorsEditorSystemComponent>(),
                azrtti_typeid<ROS2EditorCameraSystemComponent>(),
                azrtti_typeid<LidarRegistrarEditorSystemComponent>(),
            };
        }
    };
} // namespace ROS2Sensors

#if defined(O3DE_GEM_NAME)
AZ_DECLARE_MODULE_CLASS(AZ_JOIN(Gem_, O3DE_GEM_NAME, _Editor), ROS2Sensors::ROS2SensorsEditorModule)
#else
AZ_DECLARE_MODULE_CLASS(Gem_ROS2Sensors_Editor, ROS2Sensors::ROS2SensorsEditorModule)
#endif
