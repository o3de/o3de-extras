/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ROS2SensorsModuleInterface.h"
#include <AzCore/Memory/Memory.h>

#include <Clients/ROS2SensorsSystemComponent.h>
#include <ROS2Sensors/ROS2SensorsTypeIds.h>

#include <Camera/PostProcessing/ROS2ImageEncodingConversionComponent.h>
#include <Camera/ROS2CameraSensorComponent.h>
#include <Camera/ROS2CameraSystemComponent.h>
#include <GNSS/ROS2GNSSSensorComponent.h>
#include <Imu/ROS2ImuSensorComponent.h>
#include <Lidar/ClassSegmentationConfigurationComponent.h>
#include <Lidar/LidarRegistrarSystemComponent.h>
#include <Lidar/ROS2Lidar2DSensorComponent.h>
#include <Lidar/ROS2LidarSensorComponent.h>
#include <Odometry/ROS2OdometrySensorComponent.h>
#ifdef WITH_GAZEBO_MSGS
#include <ContactSensor/ROS2ContactSensorComponent.h>
#endif

namespace ROS2Sensors
{
    AZ_TYPE_INFO_WITH_NAME_IMPL(ROS2SensorsModuleInterface, "ROS2SensorsModuleInterface", ROS2SensorsModuleInterfaceTypeId);
    AZ_RTTI_NO_TYPE_INFO_IMPL(ROS2SensorsModuleInterface, AZ::Module);
    AZ_CLASS_ALLOCATOR_IMPL(ROS2SensorsModuleInterface, AZ::SystemAllocator);

    ROS2SensorsModuleInterface::ROS2SensorsModuleInterface()
    {
        m_descriptors.insert(
            m_descriptors.end(),
            {
                ROS2SensorsSystemComponent::CreateDescriptor(),
                ROS2CameraSensorComponent::CreateDescriptor(),
                ROS2SystemCameraComponent::CreateDescriptor(),
                ROS2ImageEncodingConversionComponent::CreateDescriptor(),
#ifdef WITH_GAZEBO_MSGS
                ROS2ContactSensorComponent::CreateDescriptor(),
#endif
                ROS2GNSSSensorComponent::CreateDescriptor(),
                ROS2ImuSensorComponent::CreateDescriptor(),
                ROS2LidarSensorComponent::CreateDescriptor(),
                ROS2Lidar2DSensorComponent::CreateDescriptor(),
                ClassSegmentationConfigurationComponent::CreateDescriptor(),
                LidarRegistrarSystemComponent::CreateDescriptor(),
                ROS2OdometrySensorComponent::CreateDescriptor(),
            });
    }

    AZ::ComponentTypeList ROS2SensorsModuleInterface::GetRequiredSystemComponents() const
    {
        return AZ::ComponentTypeList{
            azrtti_typeid<ROS2SensorsSystemComponent>(),
            azrtti_typeid<ROS2SystemCameraComponent>(),
            azrtti_typeid<LidarRegistrarSystemComponent>(),
        };
    }
} // namespace ROS2Sensors
