/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root
 * of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ROS2ControllersEditorSystemComponent.h"
#include <AzCore/Serialization/SerializeContext.h>

#include <Manipulation/Controllers/JointsArticulationControllerComponent.h>
#include <Manipulation/Controllers/JointsPIDControllerComponent.h>
#include <Manipulation/JointsManipulationEditorComponent.h>
#include <Manipulation/JointsTrajectoryComponent.h>
#include <ROS2Controllers/ROS2ControllersTypeIds.h>
#include <RobotControl/Controllers/AckermannController/AckermannControlComponent.h>
#include <RobotControl/Controllers/SkidSteeringController/SkidSteeringControlComponent.h>
#include <RobotControl/ROS2RobotControlComponent.h>
#include <VehicleDynamics/ModelComponents/AckermannModelComponent.h>
#include <VehicleDynamics/ModelComponents/SkidSteeringModelComponent.h>
#include <VehicleDynamics/ModelLimits/AckermannModelLimits.h>
#include <VehicleDynamics/WheelControllerComponent.h>

namespace ROS2Controllers
{
    AZ_COMPONENT_IMPL(
        ROS2ControllersEditorSystemComponent,
        "ROS2ControllersEditorSystemComponent",
        ROS2ControllersEditorSystemComponentTypeId,
        BaseSystemComponent);

    void ROS2ControllersEditorSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<ROS2ControllersEditorSystemComponent, ROS2ControllersSystemComponent>()->Version(1)->Attribute(
                AZ::Edit::Attributes::SystemComponentTags, AZStd::vector<AZ::Crc32>({ AZ_CRC_CE("AssetBuilder") }));
        }
    }

    ROS2ControllersEditorSystemComponent::ROS2ControllersEditorSystemComponent()
    {
        if (ROS2ControllersEditorInterface::Get() == nullptr)
        {
            ROS2ControllersEditorInterface::Register(this);
        }
    }

    ROS2ControllersEditorSystemComponent::~ROS2ControllersEditorSystemComponent()
    {
        if (ROS2ControllersEditorInterface::Get() == this)
        {
            ROS2ControllersEditorInterface::Unregister(this);
        }
    }

    void ROS2ControllersEditorSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        BaseSystemComponent::GetProvidedServices(provided);
        provided.push_back(AZ_CRC_CE("ROS2ControllersEditorService"));
    }

    void ROS2ControllersEditorSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        BaseSystemComponent::GetIncompatibleServices(incompatible);
        incompatible.push_back(AZ_CRC_CE("ROS2ControllersEditorService"));
    }

    void ROS2ControllersEditorSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        BaseSystemComponent::GetRequiredServices(required);
    }

    void ROS2ControllersEditorSystemComponent::GetDependentServices(
        [[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
        BaseSystemComponent::GetDependentServices(dependent);
    }

    void ROS2ControllersEditorSystemComponent::Activate()
    {
        ROS2ControllersSystemComponent::Activate();
        AzToolsFramework::EditorEvents::Bus::Handler::BusConnect();
    }

    void ROS2ControllersEditorSystemComponent::Deactivate()
    {
        AzToolsFramework::EditorEvents::Bus::Handler::BusDisconnect();
        ROS2ControllersSystemComponent::Deactivate();
    }

    AZ::Component* ROS2ControllersEditorSystemComponent::CreateWheelControllerComponent(
        AZ::Entity& entity, const AZ::EntityId& steeringEntity, const float steeringScale)
    {
        return CreateComponent<VehicleDynamics::WheelControllerComponent>(entity, steeringEntity, steeringScale);
    }

    AZ::Component* ROS2ControllersEditorSystemComponent::CreateROS2RobotControlComponent(
        AZ::Entity& entity, const ControlConfiguration& configuration)
    {
        return CreateComponent<ROS2Controllers::ROS2RobotControlComponent>(entity, configuration);
    }

    AZ::Component* ROS2ControllersEditorSystemComponent::CreateAckermannVehicleModelComponent(
        AZ::Entity& entity,
        const VehicleDynamics::VehicleConfiguration& configuration,
        const float speedLimit,
        const float steeringLimit,
        const float acceleration,
        const PidConfiguration& steeringPid)
    {
        VehicleDynamics::AckermannModelLimits modelLimits(speedLimit, steeringLimit, acceleration);

        return CreateComponent<VehicleDynamics::AckermannVehicleModelComponent>(
            entity, configuration, VehicleDynamics::AckermannDriveModel(modelLimits, steeringPid));
    }

    AZ::Component* ROS2ControllersEditorSystemComponent::CreateAckermannControlComponent(AZ::Entity& entity)
    {
        return CreateComponent<AckermannControlComponent>(entity);
    }

    AZ::Component* ROS2ControllersEditorSystemComponent::CreateSkidSteeringModelComponent(
        AZ::Entity& entity,
        const VehicleDynamics::VehicleConfiguration& configuration,
        const float linearLimit,
        const float angularLimit,
        const float linearAcceleration,
        const float angularAcceleration)
    {
        VehicleDynamics::SkidSteeringModelLimits modelLimits(linearLimit, angularLimit, linearAcceleration, angularAcceleration);

        return CreateComponent<VehicleDynamics::SkidSteeringModelComponent>(
            entity, configuration, VehicleDynamics::SkidSteeringDriveModel(modelLimits));
    }

    AZ::Component* ROS2ControllersEditorSystemComponent::CreateSkidSteeringControlComponent(AZ::Entity& entity)
    {
        return CreateComponent<ROS2Controllers::SkidSteeringControlComponent>(entity);
    }

    AZ::Component* ROS2ControllersEditorSystemComponent::CreateJointsArticulationControllerComponent(AZ::Entity& entity)
    {
        return CreateComponent<ROS2Controllers::JointsArticulationControllerComponent>(entity);
    }

    AZ::Component* ROS2ControllersEditorSystemComponent::CreateJointsPIDControllerComponent(AZ::Entity& entity)
    {
        return CreateComponent<ROS2Controllers::JointsPIDControllerComponent>(entity);
    }

    AZ::Component* ROS2ControllersEditorSystemComponent::CreateJointsManipulationEditorComponent(
        AZ::Entity& entity, const ROS2::PublisherConfiguration& publisherConfig)
    {
        return CreateComponent<ROS2Controllers::JointsManipulationEditorComponent>(entity, publisherConfig);
    }

    AZ::Component* ROS2ControllersEditorSystemComponent::CreateJointsTrajectoryComponent(
        AZ::Entity& entity, const AZStd::string& actionName)
    {
        return CreateComponent<ROS2Controllers::JointsTrajectoryComponent>(entity, actionName);
    }
} // namespace ROS2Controllers
