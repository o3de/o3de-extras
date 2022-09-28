/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "RigidBodyTwistControlComponent.h"
#include <AzCore/Component/TransformBus.h>
#include <AzCore/Math/MathUtils.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>
#include <AzFramework/Physics/RigidBodyBus.h>

namespace ROS2
{
    void RigidBodyTwistControlComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<RigidBodyTwistControlComponent, AZ::Component>()->Version(1);
            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<RigidBodyTwistControlComponent>("Rigid Body Twist Control", "Simple control through RigidBody")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2");
            }
        }
    }

    void RigidBodyTwistControlComponent::Activate()
    {
        TwistNotificationBus::Handler::BusConnect();
    }

    void RigidBodyTwistControlComponent::Deactivate()
    {
        TwistNotificationBus::Handler::BusDisconnect();
    }

    void RigidBodyTwistControlComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC("ROS2RobotControl"));
        required.push_back(AZ_CRC_CE("PhysicsRigidBodyService"));
    }

    void RigidBodyTwistControlComponent::TwistReceived(const AZ::Vector3& linear, const AZ::Vector3& angular)
    {
        auto thisEntityId = GetEntityId();
        // Get current linear velocity
        AZ::Vector3 currentLinearVelocity;
        Physics::RigidBodyRequestBus::EventResult(currentLinearVelocity, thisEntityId, &Physics::RigidBodyRequests::GetLinearVelocity);

        // Convert local steering to world frame
        AZ::Transform robotTransform;
        AZ::TransformBus::EventResult(robotTransform, thisEntityId, &AZ::TransformBus::Events::GetWorldTM);
        auto transformedLinearVelocity = robotTransform.TransformVector(linear);

        // Overwrite control velocities on two axis
        currentLinearVelocity.SetX(transformedLinearVelocity.GetX());
        currentLinearVelocity.SetY(transformedLinearVelocity.GetY());

        // Reapply desired velocities
        Physics::RigidBodyRequestBus::Event(thisEntityId, &Physics::RigidBodyRequests::SetLinearVelocity, currentLinearVelocity);
        Physics::RigidBodyRequestBus::Event(thisEntityId, &Physics::RigidBodyRequests::SetAngularVelocity, angular);
    }
} // namespace ROS2
