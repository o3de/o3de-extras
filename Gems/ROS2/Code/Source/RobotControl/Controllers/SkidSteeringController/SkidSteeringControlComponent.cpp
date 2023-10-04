/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "SkidSteeringControlComponent.h"
#include <AzCore/Component/TransformBus.h>
#include <AzCore/Math/MathUtils.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>
#include <AzFramework/Physics/RigidBodyBus.h>
#include <PhysX/Joint/PhysXJointRequestsBus.h>
#include <ROS2/VehicleDynamics/VehicleInputControlBus.h>
#include <VehicleDynamics/WheelControllerComponent.h>

namespace ROS2
{
    void SkidSteeringControlComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<SkidSteeringControlComponent, AZ::Component>()->Version(1);
            if (AZ::EditContext* editorContext = serialize->GetEditContext())
            {
                editorContext->Class<SkidSteeringControlComponent>("Skid Steering Twist Control", "")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->Attribute(AZ::Edit::Attributes::Icon, "Editor/Icons/Components/SkidSteeringTwistControl.svg")
                    ->Attribute(AZ::Edit::Attributes::ViewportIcon, "Editor/Icons/Components/Viewport/SkidSteeringTwistControl.svg");
            }
        }
    }

    void SkidSteeringControlComponent::Activate()
    {
        TwistNotificationBus::Handler::BusConnect(GetEntityId());
    }

    void SkidSteeringControlComponent::Deactivate()
    {
        TwistNotificationBus::Handler::BusDisconnect();
    }

    void SkidSteeringControlComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("ROS2RobotControl"));
        required.push_back(AZ_CRC_CE("SkidSteeringModelService"));
    }

    void SkidSteeringControlComponent::TwistReceived(const AZ::Vector3& linear, const AZ::Vector3& angular)
    {
        // Notify input system for vehicle dynamics. Only speed and steering is currently supported.
        VehicleDynamics::VehicleInputControlRequestBus::Event(
            GetEntityId(), &VehicleDynamics::VehicleInputControlRequests::SetTargetLinearSpeedV3, linear);
        VehicleDynamics::VehicleInputControlRequestBus::Event(
            GetEntityId(), &VehicleDynamics::VehicleInputControlRequests::SetTargetAngularSpeedV3, angular);
    }
} // namespace ROS2
