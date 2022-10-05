/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "AckermannControlComponent.h"
#include "VehicleDynamics/VehicleInputControlBus.h"
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>
#include <AzFramework/Physics/RigidBodyBus.h>

namespace ROS2
{
    void AckermannControlComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<AckermannControlComponent, AZ::Component>()->Version(1);
            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<AckermannControlComponent>("Ackermann Control", "Relays Ackermann commands to vehicle inputs")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2");
            }
        }
    }

    void AckermannControlComponent::Activate()
    {
        AckermannNotificationBus::Handler::BusConnect(GetEntityId());
    }
    void AckermannControlComponent::Deactivate()
    {
        AckermannNotificationBus::Handler::BusDisconnect();
    }

    void AckermannControlComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC("ROS2RobotControl"));
        required.push_back(AZ_CRC("VehicleModelService"));
    }

    void AckermannControlComponent::AckermannReceived(const AckermannCommandStruct& acs)
    {
        // Notify input system for vehicle dynamics. Only speed and steering is currently supported.
        VehicleDynamics::VehicleInputControlRequestBus::Event(
            GetEntityId(), &VehicleDynamics::VehicleInputControlRequests::SetTargetLinearSpeed, acs.m_speed);
        VehicleDynamics::VehicleInputControlRequestBus::Event(
            GetEntityId(), &VehicleDynamics::VehicleInputControlRequests::SetTargetSteering, acs.m_steeringAngle);
    }
} // namespace ROS2
