/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "RobotControl/TwistControl/TwistControl.h"
#include "RobotControl/TwistControl/TwistBus.h"
#include "Utilities/ROS2Conversions.h"
#include <AzCore/Component/TransformBus.h>
#include <AzCore/Math/MathUtils.h>
#include <AzFramework/Physics/RigidBodyBus.h>

namespace ROS2
{
    void TwistControl::BroadcastBus(const geometry_msgs::msg::Twist& message)
    {
        TwistNotificationBus::Broadcast(
            &TwistNotifications::TwistReceived,
            ROS2Conversions::FromROS2Vector3(message.linear),
            ROS2Conversions::FromROS2Vector3(message.angular));
    }

    void TwistControl::ApplyControl(const geometry_msgs::msg::Twist& message)
    {
        /*
        This is an example of control implementation which sets the desired velocity on a single body.
        To imitate the steering, current linear and angular velocities of a single rigidbody are forcefully overwritten
        with the desired control.
        TODO: Control the robot with forces applied to the wheels instead of directly setting up body velocity.
        */

        // Check the body entity id (it might be not set at all)
        auto body = m_controlConfiguration.m_robotConfiguration.m_body;
        if (!body.IsValid())
        {
            AZ_ErrorOnce("TwistControl", false, "Invalid body component for twist control.");
        }

        // Convert steering from ROS2 to O3DE coordinate system
        const AZ::Vector3 linearVelocity = ROS2Conversions::FromROS2Vector3(message.linear);
        const AZ::Vector3 angularVelocity = ROS2Conversions::FromROS2Vector3(message.angular);

        // Get current linear velocity
        AZ::Vector3 currentLinearVelocity;
        Physics::RigidBodyRequestBus::EventResult(currentLinearVelocity, body, &Physics::RigidBodyRequests::GetLinearVelocity);

        // Convert local steering to world frame
        AZ::Transform robotTransform;
        AZ::TransformBus::EventResult(robotTransform, body, &AZ::TransformBus::Events::GetWorldTM);
        auto transformedLinearVelocity = robotTransform.TransformVector(linearVelocity);

        // Overwrite control velocities on two axis
        currentLinearVelocity.SetX(transformedLinearVelocity.GetX());
        currentLinearVelocity.SetY(transformedLinearVelocity.GetY());

        // Reapply desired velocities
        Physics::RigidBodyRequestBus::Event(body, &Physics::RigidBodyRequests::SetLinearVelocity, currentLinearVelocity);
        Physics::RigidBodyRequestBus::Event(body, &Physics::RigidBodyRequests::SetAngularVelocity, angularVelocity);
    }
} // namespace ROS2
