/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <ROS2/Frame/ROS2FrameComponent.h>
#include <ROS2/Manipulation/Controllers/ManipulatorJointTrajectoryComponent.h>
#include <ROS2/ROS2Bus.h>

namespace ROS2
{
    ManipulatorJointTrajectoryComponent::ManipulatorJointTrajectoryComponent(
        const AZStd::string& actionName, const AZ::EntityId& entityId)
        : m_entityId(entityId)
    {
        auto ros2Node = ROS2::ROS2Interface::Get()->GetNode();
        auto ros2Frame = GetEntityId()->FindComponent<ROS2FrameComponent>();
        AZ_Assert(ros2Frame, "Missing Frame Component!");
        AZStd::string namespacedAction = ROS2Names::GetNamespacedName(ros2Frame->GetNamespace(), actionName);
        m_followTrajectoryServer = AZStd::make_unique<FollowJointTrajectoryActionServer>(namespacedAction);
    }

    void ManipulatorJointTrajectoryComponent::GetEntityId() const
    {
        return m_entityId;
    }

    void ManipulatorJointTrajectoryComponent::SetTrajectoryGoal(const ManipulatorTrajectoryRequestBus::TrajectoryGoal& trajectoryGoal)
    {
        m_trajectoryGoal = trajectoryGoal;
        m_trajectoryExecutionStartTime = rclcpp::Time(ROS2::ROS2Interface::Get()->GetROSTimestamp());
        m_trajectoryInProgress = true;
    }

    void ManipulatorJointTrajectoryComponent::CancelTrajectoryGoal()
    {
        m_followTrajectoryServer->CancelGoal();
        // TODO - erase trajectory?
        m_trajectoryInProgress = false;
    }

    void ManipulatorJointTrajectoryComponent::FollowTrajectory(const uint64_t deltaTimeNs)
    {
        if (GetGoalStatus() != GoalStatus::Executing)
        { // Trajectory was cancelled, already succeeded or not yet executing.
            KeepStillPosition(deltaTimeNs);
            return;
        }

        if (!m_trajectoryInProgress)
        { // No trajectory to follow.
            return;
        }

        if (m_trajectoryGoal.trajectory.points.size() == 0)
        { // The manipulator has reached the goal.
            AZ_TracePrintf("ManipulatorComponent", "Goal Concluded: all points reached");
            auto successResult = std::make_shared<FollowJointTrajectory::Result>(); // Empty result defaults to success.
            m_followTrajectoryServer->GoalSuccess(successResult);
            m_trajectoryInProgress = false;
            return;
        }

        auto desiredGoal = trajectory.points.front();
        rclcpp::Duration targetGoalTime = rclcpp::Duration(desiredGoal.time_from_start); //!< requested arrival time for trajectory point.
        rclcpp::Time timeNow = rclcpp::Time(ROS2::ROS2Interface::Get()->GetROSTimestamp()); //!< current simulation time.
        rclcpp::Duration threshold = rclcpp::Duration::from_nanoseconds(1e7);

        if (m_trajectoryExecutionStartTime + targetGoalTime <= timeNow + threshold)
        { // Jump to the next point if current simulation time is ahead of timeFromStart
            m_trajectory.trajectory.points.erase(m_trajectory.trajectory.points.begin());
            FollowTrajectory(deltaTimeNs);
            return;
        }

        MoveToNextPoint(desiredGoal, deltaTimeNs, timeNow);
    }

    void ManipulatorJointTrajectoryComponent::MoveToNextPoint(
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




    void ManipulatorComponent::InitializePid()
    {
        for (auto& pid : m_pidConfigurationVector)
        {
            pid.InitializePid();
        }
    }

    void ManipulatorComponent::InitializeCurrentPosition()
    {
        auto* jointPublisherComponent = GetEntity()->FindComponent<JointPublisherComponent>();
        if (!jointPublisherComponent)
        {
            for (auto& [jointName, hingeComponent] : jointPublisherComponent->GetHierarchyMap())
            {
                m_jointKeepStillPosition[jointName] = jointPublisherComponent->GetJointPosition(jointName);
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

    float ManipulatorComponent::ComputeFFJointVelocity(
        const float currentPosition, const float desiredPosition, const rclcpp::Duration& duration) const
    {
        // FeedForward (dummy) method
        float desiredVelocity = (desiredPosition - currentPosition) / duration.seconds();
        return desiredVelocity;
    }

    float ManipulatorComponent::ComputePIDJointVelocity(
        const float currentPosition, const float desiredPosition, const uint64_t& deltaTimeNs, int& jointIndex)
    {
        // PID method
        float error = desiredPosition - currentPosition;
        AZ_Warning("ManipulatorComponent", jointIndex < m_pidConfigurationVector.size(), "Joint index out of range");
        if (jointIndex < m_pidConfigurationVector.size())
        {
            float command = m_pidConfigurationVector.at(jointIndex).ComputeCommand(error, deltaTimeNs);
            return command;
        }
        return 0;
    }

    void ManipulatorComponent::SetJointVelocity(const AZ::EntityComponentIdPair idPair, const float desiredVelocity)
    {
        PhysX::JointRequestBus::Event(idPair, &PhysX::JointRequests::SetVelocity, desiredVelocity);
    }

    void ManipulatorComponent::KeepStillPosition([[maybe_unused]] const uint64_t deltaTimeNs)
    {
        if (!m_keepStillPositionInitialize)
        {
            InitializeCurrentPosition();
            m_keepStillPositionInitialize = true;
        }

        int jointIndex = 0;
        for (auto& [jointName, desiredPosition] : m_jointKeepStillPosition)
        {
            if (jointIndex >= m_pidConfigurationVector.size())
            {
                break;
            }
            auto* jointPublisherComponent = GetEntity()->FindComponent<JointPublisherComponent>();
            if (jointPublisherComponent)
            {
                AZ_Assert(
                    jointPublisherComponent->GetHierarchyMap().contains(jointName),
                    "Joint name %s not found in the hierarchy map",
                    AZStd::string(jointName.GetStringView()).c_str());
                float currentPosition = jointPublisherComponent->GetJointPosition(jointName);
                float desiredVelocity = 0.0f;
                if (m_controllerType == Controller::FeedForward)
                {
                    desiredVelocity = ComputeFFJointVelocity(
                        currentPosition,
                        desiredPosition,
                        rclcpp::Duration::from_nanoseconds(5e8)); // Dummy forward time reference
                }
                else if (m_controllerType == Controller::PID)
                {
                    desiredVelocity = ComputePIDJointVelocity(currentPosition, desiredPosition, deltaTimeNs, jointIndex);
                }
                SetJointVelocity(jointPublisherComponent->GetHierarchyMap().at(jointName), desiredVelocity);
                jointIndex++;
            }
        }
    }
} // namespace ROS2
