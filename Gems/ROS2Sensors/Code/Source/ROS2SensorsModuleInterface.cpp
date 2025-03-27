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
#include <ROS2/Sensor/Events/PhysicsBasedSource.h>
#include <ROS2/Sensor/Events/TickBasedSource.h>

#include <ROS2Sensors/ROS2SensorsTypeIds.h>
#include <ROS2Sensors/Sensor/ROS2SensorComponentBase.h>

// TEMP #include <Camera/PostProcessing/ROS2ImageEncodingConversionComponent.h>
// TEMP #include <Camera/ROS2CameraSensorComponent.h>
// TEMP #include <Camera/ROS2CameraSystemComponent.h>
#include <ContactSensor/ROS2ContactSensorComponent.h>
#include <GNSS/ROS2GNSSSensorComponent.h>
#include <Imu/ROS2ImuSensorComponent.h>
// TEMP #include <Lidar/ClassSegmentationConfigurationComponent.h>
// TEMP #include <Lidar/LidarRegistrarSystemComponent.h>
// TEMP #include <Lidar/ROS2Lidar2DSensorComponent.h>
// TEMP #include <Lidar/ROS2LidarSensorComponent.h>
#include <Odometry/ROS2OdometrySensorComponent.h>
#include <Odometry/ROS2WheelOdometry.h>

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
                ROS2::ROS2SensorComponentBase<ROS2::TickBasedSource>::CreateDescriptor(),
                ROS2::ROS2SensorComponentBase<ROS2::PhysicsBasedSource>::CreateDescriptor(),
                // TEMP ROS2CameraSensorComponent::CreateDescriptor(),
                // TEMP ROS2ImageEncodingConversionComponent::CreateDescriptor(),
                ROS2::ROS2ContactSensorComponent::CreateDescriptor(),
                // TEMP ROS2SystemCameraComponent::CreateDescriptor(),
                ROS2::ROS2GNSSSensorComponent::CreateDescriptor(),
                ROS2::ROS2ImuSensorComponent::CreateDescriptor(),
                // TEMP LidarRegistrarSystemComponent::CreateDescriptor(),
                // TEMP ROS2LidarSensorComponent::CreateDescriptor(),
                // TEMP ROS2Lidar2DSensorComponent::CreateDescriptor(),
                // TEMP ClassSegmentationConfigurationComponent::CreateDescriptor(),
                ROS2::ROS2OdometrySensorComponent::CreateDescriptor(),
                ROS2::ROS2WheelOdometryComponent::CreateDescriptor(),
            });
    }

    AZ::ComponentTypeList ROS2SensorsModuleInterface::GetRequiredSystemComponents() const
    {
        return AZ::ComponentTypeList{
            azrtti_typeid<ROS2SensorsSystemComponent>(),
            // TEMP azrtti_typeid<ROS2SystemCameraComponent>(),
            // TEMP azrtti_typeid<LidarRegistrarSystemComponent>(),
        };
    }
} // namespace ROS2Sensors
