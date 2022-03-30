/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <AzFramework/Physics/RigidBodyBus.h>
#include "TwistControl.h"
#include "Utilities/ROS2Conversions.h"

namespace ROS2
{
    void TwistControl::SetTargetComponent(const AZ::Entity* entity)
    {
        m_entityID = entity->GetId();
    }

    void TwistControl::ApplyControl(const geometry_msgs::msg::Twist& message)
    {
        const AZ::Vector3 linearVelocity = ROS2Conversions::FromROS2Vector3(message.linear);
        const AZ::Vector3 angularVelocity = ROS2Conversions::FromROS2Vector3(message.angular);
        Physics::RigidBodyRequestBus::Event(m_entityID, &Physics::RigidBodyRequests::SetLinearVelocity, linearVelocity);
        Physics::RigidBodyRequestBus::Event(m_entityID, &Physics::RigidBodyRequests::SetAngularVelocity, angularVelocity);
    }
}  // namespace ROS2