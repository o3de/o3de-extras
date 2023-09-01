/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ManipulationUtils.h"
#include <PhysX/ArticulationJointBus.h>
#include <PhysX/Joint/PhysXJointRequestsBus.h>

namespace ROS2::Utils
{
    JointStateData GetJointState(const JointInfo& jointInfo)
    {
        JointStateData result;

        if (jointInfo.m_isArticulation)
        {
            PhysX::ArticulationJointRequestBus::Event(
                jointInfo.m_entityComponentIdPair.GetEntityId(),
                [&](PhysX::ArticulationJointRequests* articulationJointRequests)
                {
                    result.position = articulationJointRequests->GetJointPosition(jointInfo.m_axis);
                    result.velocity = articulationJointRequests->GetJointVelocity(jointInfo.m_axis);
                    const bool is_acceleration_driven = articulationJointRequests->IsAccelerationDrive(jointInfo.m_axis);
                    if (!is_acceleration_driven)
                    {
                        const float stiffness = articulationJointRequests->GetDriveStiffness(jointInfo.m_axis);
                        const float damping = articulationJointRequests->GetDriveDamping(jointInfo.m_axis);
                        const float targetPosition = articulationJointRequests->GetDriveTarget(jointInfo.m_axis);
                        const float targetVelocity = articulationJointRequests->GetDriveTargetVelocity(jointInfo.m_axis);
                        const float maxEffort = articulationJointRequests->GetMaxForce(jointInfo.m_axis);
                        result.effort = stiffness * -(result.position - targetPosition) + damping * (targetVelocity - result.velocity);
                        result.effort = AZ::GetClamp(result.effort, -maxEffort, maxEffort);
                    }
                });
        }
        else
        {
            PhysX::JointRequestBus::Event(
                jointInfo.m_entityComponentIdPair,
                [&](PhysX::JointRequests* jointRequests)
                {
                    result.position = jointRequests->GetPosition();
                    result.velocity = jointRequests->GetVelocity();
                });
        }
        return result;
    }
} // namespace ROS2::Utils
