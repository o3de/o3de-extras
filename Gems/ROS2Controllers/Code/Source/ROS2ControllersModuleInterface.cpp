/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root
 * of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ROS2ControllersModuleInterface.h"
#include <AzCore/Memory/Memory.h>

#include <ROS2Controllers/ROS2ControllersTypeIds.h>

#include <Clients/ROS2ControllersSystemComponent.h>

#include <Gripper/FingerGripperComponent.h>
#include <Gripper/GripperActionServerComponent.h>
#include <Gripper/VacuumGripperComponent.h>
#include <Manipulation/Controllers/JointsArticulationControllerComponent.h>
#include <Manipulation/Controllers/JointsPIDControllerComponent.h>
#include <Manipulation/JointsManipulationComponent.h>
#include <Manipulation/JointsPositionsComponent.h>
#include <Manipulation/JointsTrajectoryComponent.h>
#include <ROS2Controllers/Manipulation/MotorizedJoints/JointMotorControllerComponent.h>
#include <ROS2Controllers/Manipulation/MotorizedJoints/ManualMotorControllerComponent.h>
#include <ROS2Controllers/Manipulation/MotorizedJoints/PidMotorControllerComponent.h>
#include <RobotControl/Controllers/AckermannController/AckermannControlComponent.h>
#include <RobotControl/Controllers/RigidBodyController/RigidBodyTwistControlComponent.h>
#include <RobotControl/Controllers/SkidSteeringController/SkidSteeringControlComponent.h>
#include <RobotControl/ROS2RobotControlComponent.h>
#include <VehicleDynamics/ModelComponents/AckermannModelComponent.h>
#include <VehicleDynamics/ModelComponents/SkidSteeringModelComponent.h>
#include <VehicleDynamics/WheelControllerComponent.h>

namespace ROS2Controllers
{
    AZ_TYPE_INFO_WITH_NAME_IMPL(ROS2ControllersModuleInterface, "ROS2ControllersModuleInterface", ROS2ControllersModuleInterfaceTypeId);
    AZ_RTTI_NO_TYPE_INFO_IMPL(ROS2ControllersModuleInterface, AZ::Module);
    AZ_CLASS_ALLOCATOR_IMPL(ROS2ControllersModuleInterface, AZ::SystemAllocator);

    ROS2ControllersModuleInterface::ROS2ControllersModuleInterface()
    {
        m_descriptors.insert(
            m_descriptors.end(),
            {
                ROS2ControllersSystemComponent::CreateDescriptor(),
                GripperActionServerComponent::CreateDescriptor(),
                FingerGripperComponent::CreateDescriptor(),
                VacuumGripperComponent::CreateDescriptor(),
                JointsArticulationControllerComponent::CreateDescriptor(),
                JointsPIDControllerComponent::CreateDescriptor(),
                JointsManipulationComponent::CreateDescriptor(),
                JointsPositionsComponent::CreateDescriptor(),
                JointsTrajectoryComponent::CreateDescriptor(),
                JointMotorControllerComponent::CreateDescriptor(),
                ManualMotorControllerComponent::CreateDescriptor(),
                AckermannControlComponent::CreateDescriptor(),
                RigidBodyTwistControlComponent::CreateDescriptor(),
                SkidSteeringControlComponent::CreateDescriptor(),
                ROS2RobotControlComponent::CreateDescriptor(),
                VehicleDynamics::AckermannVehicleModelComponent::CreateDescriptor(),
                VehicleDynamics::SkidSteeringModelComponent::CreateDescriptor(),
                VehicleDynamics::WheelControllerComponent::CreateDescriptor(),
            });
    }

    AZ::ComponentTypeList ROS2ControllersModuleInterface::GetRequiredSystemComponents() const
    {
        return AZ::ComponentTypeList{
            azrtti_typeid<ROS2ControllersSystemComponent>(),
        };
    }
} // namespace ROS2Controllers
