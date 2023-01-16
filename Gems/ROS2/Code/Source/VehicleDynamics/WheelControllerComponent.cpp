/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "WheelControllerComponent.h"
#include <AzCore/Debug/Trace.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>
#include <AzCore/Serialization/SerializeContext.h>

namespace ROS2::VehicleDynamics
{
    void WheelControllerComponent::Activate()
    {
    }

    void WheelControllerComponent::Deactivate()
    {
    }

    void WheelControllerComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("PhysicsRigidBodyService"));
    }

    void WheelControllerComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("WheelControllerService"));
    }

    void WheelControllerComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        // Only one per entity
        incompatible.push_back(AZ_CRC_CE("WheelControllerService"));
    }

    void WheelControllerComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<WheelControllerComponent, AZ::Component>()
                ->Version(2)
                ->Field("SteeringEntity", &WheelControllerComponent::m_steeringEntity)
                ->Field("DriveDir", &WheelControllerComponent::m_driveDir)
                ->Field("SteeringDir", &WheelControllerComponent::m_steeringDir);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<WheelControllerComponent>("Wheel controller", "Handle wheel physics")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &WheelControllerComponent::m_steeringEntity,
                        "Steering entity",
                        "Entity which steers the wheel - typically a parent entity")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &WheelControllerComponent::m_driveDir,
                        "Direction of drive axis",
                        "The direction of torque applied to the wheel entity")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &WheelControllerComponent::m_steeringDir,
                        "Direction of steering axis",
                        "The direction of torque applied to the steering entity");
            }
        }
    }
} // namespace ROS2::VehicleDynamics
