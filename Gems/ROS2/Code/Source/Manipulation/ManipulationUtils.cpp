/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ManipulationUtils.h"

namespace ROS2::Utils
{
    JointStateData GetJointState(const JointInfo& jointInfo)
    {
        JointStateData result;
        float position, velocity, effort;

        if (jointInfo.m_isArticulation)
        {
            float stiffness{ 0.f }, damping{ 0.f }, target_position{ 0.f }, target_velocity{ 0.f }, max_effort{ 0.f };
            bool is_acceleration_driven;
            PhysX::ArticulationJointRequestBus::Event(
                jointInfo.m_entityComponentIdPair.GetEntityId(),
                [&](PhysX::ArticulationJointRequests* articulationJointRequests)
                {
                    stiffness = articulationJointRequests->GetDriveStiffness(jointInfo.m_axis);
                    damping = articulationJointRequests->GetDriveDamping(jointInfo.m_axis);
                    target_position = articulationJointRequests->GetDriveTarget(jointInfo.m_axis);
                    position = articulationJointRequests->GetJointPosition(jointInfo.m_axis);
                    target_velocity = articulationJointRequests->GetDriveTargetVelocity(jointInfo.m_axis);
                    velocity = articulationJointRequests->GetJointVelocity(jointInfo.m_axis);
                    max_effort = articulationJointRequests->GetMaxForce(jointInfo.m_axis);
                    is_acceleration_driven = articulationJointRequests->IsAccelerationDrive(jointInfo.m_axis);
                });
            if (!is_acceleration_driven)
            {
                effort = stiffness * -(position - target_position) + damping * (target_velocity - velocity);
                effort = AZ::GetClamp(effort, -max_effort, max_effort);
            }
            else
            {
                effort = 0.f;
            }
        }
        else
        {
            PhysX::JointRequestBus::Event(
                jointInfo.m_entityComponentIdPair,
                [&](PhysX::JointRequests* jointRequests)
                {
                    position = jointRequests->GetPosition();
                    velocity = jointRequests->GetVelocity();
                });
            effort = 0.f;
        }
        result.position = position;
        result.velocity = velocity;
        result.effort = effort;
        return result;
    }
} // namespace ROS2::Utils