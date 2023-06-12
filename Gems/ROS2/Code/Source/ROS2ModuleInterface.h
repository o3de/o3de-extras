/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include "ROS2SystemComponent.h"
#include <AzCore/Memory/SystemAllocator.h>
#include <AzCore/Module/Module.h>
#include <Camera/ROS2CameraSensorComponent.h>
#include <GNSS/ROS2GNSSSensorComponent.h>
#include <Lidar/LidarRegistrarSystemComponent.h>
#include <Lidar/ROS2Lidar2DSensorComponent.h>
#include <Lidar/ROS2LidarSensorComponent.h>
#include <Odometry/ROS2OdometrySensorComponent.h>
#include <Odometry/ROS2WheelOdometry.h>
#include <ROS2/Frame/ROS2FrameComponent.h>
#include <ROS2/Imu/ROS2ImuSensorComponent.h>
#include <ROS2/Manipulation/JointMotorControllerComponent.h>
#include <ROS2/Manipulation/JointPublisherComponent.h>
#include <ROS2/Manipulation/ManipulatorControllerComponent.h>
#include <ROS2/Manipulation/ManualMotorControllerComponent.h>
#include <ROS2/Manipulation/PidMotorControllerComponent.h>
#include <RobotControl/Controllers/AckermannController/AckermannControlComponent.h>
#include <RobotControl/Controllers/RigidBodyController/RigidBodyTwistControlComponent.h>
#include <RobotControl/Controllers/SkidSteeringController/SkidSteeringControlComponent.h>
#include <RobotControl/ROS2RobotControlComponent.h>
#include <RobotImporter/ROS2RobotImporterSystemComponent.h>
#include <SimulationUtils/FollowingCameraComponent.h>
#include <Spawner/ROS2SpawnPointComponent.h>
#include <Spawner/ROS2SpawnerComponent.h>
#include <VehicleDynamics/ModelComponents/AckermannModelComponent.h>
#include <VehicleDynamics/ModelComponents/SkidSteeringModelComponent.h>
#include <VehicleDynamics/VehicleModelComponent.h>
#include <VehicleDynamics/WheelControllerComponent.h>

namespace ROS2
{
    class ROS2ModuleInterface : public AZ::Module
    {
    public:
        AZ_RTTI(ROS2ModuleInterface, "{8b5567cb-1de9-49af-9cd4-9750d4abcd6b}", AZ::Module);
        AZ_CLASS_ALLOCATOR(ROS2ModuleInterface, AZ::SystemAllocator);

        ROS2ModuleInterface()
        {
            // Push results of [MyComponent]::CreateDescriptor() into m_descriptors here.
            // Add ALL components descriptors associated with this gem to m_descriptors.
            // This will associate the AzTypeInfo information for the components with the SerializeContext, BehaviorContext and EditContext.
            // This happens through the [MyComponent]::Reflect() function.
            m_descriptors.insert(
                m_descriptors.end(),
                {
                    ROS2SystemComponent::CreateDescriptor(),
                    LidarRegistrarSystemComponent::CreateDescriptor(),
                    ROS2RobotImporterSystemComponent::CreateDescriptor(),
                    ROS2SensorComponent::CreateDescriptor(),
                    ROS2ImuSensorComponent::CreateDescriptor(),
                    ROS2GNSSSensorComponent::CreateDescriptor(),
                    ROS2LidarSensorComponent::CreateDescriptor(),
                    ROS2Lidar2DSensorComponent::CreateDescriptor(),
                    ROS2OdometrySensorComponent::CreateDescriptor(),
                    ROS2WheelOdometryComponent::CreateDescriptor(),
                    ROS2FrameComponent::CreateDescriptor(),
                    ROS2RobotControlComponent::CreateDescriptor(),
                    ROS2CameraSensorComponent::CreateDescriptor(),
                    AckermannControlComponent::CreateDescriptor(),
                    RigidBodyTwistControlComponent::CreateDescriptor(),
                    SkidSteeringControlComponent::CreateDescriptor(),
                    ROS2CameraSensorComponent::CreateDescriptor(),
                    ROS2SpawnerComponent::CreateDescriptor(),
                    ROS2SpawnPointComponent::CreateDescriptor(),
                    VehicleDynamics::AckermannVehicleModelComponent::CreateDescriptor(),
                    VehicleDynamics::WheelControllerComponent::CreateDescriptor(),
                    VehicleDynamics::SkidSteeringModelComponent::CreateDescriptor(),
                    JointMotorControllerComponent::CreateDescriptor(),
                    ManualMotorControllerComponent::CreateDescriptor(),
                    JointPublisherComponent::CreateDescriptor(),
                    ManipulatorControllerComponent::CreateDescriptor(),
                    PidMotorControllerComponent::CreateDescriptor(),
                    FollowingCameraComponent::CreateDescriptor(),
                });
        }

        //! Add required SystemComponents to the SystemEntity.
        AZ::ComponentTypeList GetRequiredSystemComponents() const override
        {
            return AZ::ComponentTypeList{
                azrtti_typeid<ROS2SystemComponent>(),
                azrtti_typeid<LidarRegistrarSystemComponent>(),
                azrtti_typeid<ROS2RobotImporterSystemComponent>(),
            };
        }
    };
} // namespace ROS2
