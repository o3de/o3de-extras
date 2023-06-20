/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <ROS2/Manipulation/Controllers/ArticulationTrajectoryController.h>

namespace ROS2
{
    ArticulationTrajectoryController::ArticulationTrajectoryController(const AZStd::string& namespacedActionName)
        : ManipulatorJointTrajectoryComponent(namespacedActionName)
    {

    }

    void ArticulationTrajectoryController::MoveToNextPoint(
        const trajectory_msgs::msg::JointTrajectoryPoint currentTrajectoryPoint,
        uint64_t deltaTimeNs,
        const rclcpp::Time& simulationTimeNow)
    {
        for (int jointIndex = 0; jointIndex < m_trajectory.joint_names.size(); jointIndex++)
        {
            const auto& jointNameStr = m_trajectory.joint_names[jointIndex];
            const auto* jointPublisherComponent = GetEntity()->FindComponent<JointPublisherComponent>();

            if (!jointPublisherComponent)
            {
                continue;
            }
            const auto& hierarchy = jointPublisherComponent->GetHierarchyMap();
            const auto jointName = AZ::Name(jointNameStr.c_str());
            if (hierarchy.contains(jointName))
            {
                const AZ::EntityComponentIdPair& componentId = hierarchy.at(jointName);
                float currentPosition = jointPublisherComponent->GetJointPosition(jointName);
                float targetPosition = nextTrajectoryPoint.positions[jointIndex];
                rclcpp::Duration targetGoalTimeFromStart = rclcpp::Duration(nextTrajectoryPoint.time_from_start);
                float desiredVelocity = 0.0f;
                if (m_controllerType == Controller::FeedForward)
                {
                    desiredVelocity = ComputeFFJointVelocity(
                        currentPosition, targetPosition, m_trajectoryExecutionStartTime + targetGoalTimeFromStart - simulationTimeNow);
                    SetJointVelocity(componentId, desiredVelocity);
                }
                else if (m_controllerType == Controller::PID)
                {
                    desiredVelocity = ComputePIDJointVelocity(currentPosition, targetPosition, deltaTimeNs, jointIndex);
                    SetJointVelocity(componentId, desiredVelocity);
                }
                else if (m_controllerType == Controller::PhysXArticulation)
                {
                    const auto articulatedAxis = jointPublisherComponent->GetArticulationFreeAxis(jointName);
                    PhysX::ArticulationJointRequestBus::Event(
                        componentId.GetEntityId(), &PhysX::ArticulationJointRequests::SetDriveTarget, articulatedAxis, targetPosition);
                }
            }
            else
            {
                AZ_Warning("ManipulatorComponent", false, "Joint name %s not found in the hierarchy map", jointName.GetCStr());
            }
        }
    }

    void ManipulatorComponent::InitializePosition()
    {
        auto* jointPublisherComponent = GetEntity()->FindComponent<JointPublisherComponent>();
        if (!jointPublisherComponent)
        {
            return;
        }

        for (auto& [jointName, hingeComponent] : jointPublisherComponent->GetHierarchyMap())
        {
            float initialJointPosition = jointPublisherComponent->GetJointPosition(jointName);
            const AZStd::string_view jointNameStr(jointName.GetCStr());
            AZ_Printf("ManipulatorComponent", "Joint name: %s\n", jointNameStr.data());
            if (m_initialPositions.contains(jointNameStr))
            {
                initialJointPosition = m_initialPositions[jointNameStr];

                AZ_Printf(
                    "ManipulatorComponent",
                    "Joint name: %s initialJointPosition : %f \n",
                    jointNameStr.data(),
                    initialJointPosition);
            }
            if (m_controllerType == Controller::PhysXArticulation)
            {
                const auto articulatedAxis = jointPublisherComponent->GetArticulationFreeAxis(jointName);
                PhysX::ArticulationJointRequestBus::Event(
                    hingeComponent.GetEntityId(), &PhysX::ArticulationJointRequests::SetDriveTarget, articulatedAxis, initialJointPosition);
            }
            else
            {
                m_jointKeepStillPosition[jointName] = initialJointPosition;
            }
        }
    }

    void ManipulatorComponent::KeepStillPosition([[maybe_unused]] const uint64_t deltaTimeNs)
    {
        // Empty
    }
} // namespace ROS2
