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
#include <Source/RigidBodyComponent.h>

namespace ROS2::VehicleDynamics
{
    void WheelControllerComponent::Activate()
    {
        m_rigidBodyPtr = nullptr;
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

    AZ::Vector3 WheelControllerComponent::GetRotationVelocity()
    {
        if (m_rigidBodyPtr == nullptr)
        {
            Physics::RigidBodyRequestBus::EventResult(m_rigidBodyPtr, m_entity->GetId(), &Physics::RigidBodyRequests::GetRigidBody);
        }
        AZ_Assert(m_rigidBodyPtr, "No Rigid Body in the WheelController entity!");
        if (m_rigidBodyPtr)
        {
            const auto transform = m_rigidBodyPtr->GetTransform().GetInverse();
            const auto local = transform.TransformVector(m_rigidBodyPtr->GetAngularVelocity());
            return local;
        }
        return AZ::Vector3::CreateZero();
    }

    void WheelControllerComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<WheelControllerComponent, AZ::Component>()
                ->Version(3)
                ->Field("SteeringEntity", &WheelControllerComponent::m_steeringEntity)
                ->Field("SteeringScale", &WheelControllerComponent::m_steeringScale);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<WheelControllerComponent>("Wheel Controller", "Handle wheel physics")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->Attribute(AZ::Edit::Attributes::Icon, "Editor/Icons/Components/WheelController.svg")
                    ->Attribute(AZ::Edit::Attributes::ViewportIcon, "Editor/Icons/Components/Viewport/WheelController.svg")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &WheelControllerComponent::m_steeringEntity,
                        "Steering entity",
                        "Entity which steers the wheel - typically a parent entity")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &WheelControllerComponent::m_steeringScale,
                        "Scale of steering axis",
                        "The direction of torque applied to the steering entity");
            }
        }
    }

    WheelControllerComponent::WheelControllerComponent(const AZ::EntityId& steeringEntity, const float steeringScale)
        : m_steeringEntity(steeringEntity)
        , m_steeringScale(steeringScale)
    {
    }
} // namespace ROS2::VehicleDynamics
