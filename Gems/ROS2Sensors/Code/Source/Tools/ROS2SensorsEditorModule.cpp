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

//TEMP #include <Camera/ROS2CameraSensorEditorComponent.h>
//TEMP #include <Camera/ROS2EditorCameraSystemComponent.h>
//TEMP #include <Lidar/LidarRegistrarEditorSystemComponent.h>

namespace ROS2Sensors
{
    class ROS2SensorsEditorModule : public ROS2SensorsModuleInterface
    {
    public:
        AZ_RTTI(ROS2SensorsEditorModule, ROS2SensorsEditorModuleTypeId, ROS2SensorsModuleInterface);
        AZ_CLASS_ALLOCATOR(ROS2SensorsEditorModule, AZ::SystemAllocator);

        ROS2SensorsEditorModule()
        {
            // Push results of [MyComponent]::CreateDescriptor() into m_descriptors here.
            // Add ALL components descriptors associated with this gem to m_descriptors.
            // This will associate the AzTypeInfo information for the components with the the SerializeContext, BehaviorContext and
            // EditContext. This happens through the [MyComponent]::Reflect() function.
            m_descriptors.insert(
                m_descriptors.end(),
                {
                    ROS2SensorsEditorSystemComponent::CreateDescriptor(),
                    //TEMP ROS2CameraSensorEditorComponent::CreateDescriptor(),
                    //TEMP ROS2EditorCameraSystemComponent::CreateDescriptor(),
                    //TEMP LidarRegistrarEditorSystemComponent::CreateDescriptor(),
                });
        }

        /**
         * Add required SystemComponents to the SystemEntity.
         * Non-SystemComponents should not be added here
         */
        AZ::ComponentTypeList GetRequiredSystemComponents() const override
        {
            return AZ::ComponentTypeList{
                azrtti_typeid<ROS2SensorsEditorSystemComponent>(),
                //TEMP azrtti_typeid<ROS2EditorCameraSystemComponent>(),
                //TEMP azrtti_typeid<LidarRegistrarEditorSystemComponent>(),
            };
        }
    };
} // namespace ROS2Sensors

#if defined(O3DE_GEM_NAME)
AZ_DECLARE_MODULE_CLASS(AZ_JOIN(Gem_, O3DE_GEM_NAME, _Editor), ROS2Sensors::ROS2SensorsEditorModule)
#else
AZ_DECLARE_MODULE_CLASS(Gem_ROS2Sensors_Editor, ROS2Sensors::ROS2SensorsEditorModule)
#endif
