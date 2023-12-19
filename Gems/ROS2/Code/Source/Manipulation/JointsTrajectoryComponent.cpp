/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "JointsTrajectoryComponent.h"
#include <AzCore/Serialization/EditContext.h>
#include <PhysX/ArticulationJointBus.h>
#include <ROS2/Frame/ROS2FrameComponent.h>
#include <ROS2/Manipulation/JointsManipulationRequests.h>
#include <ROS2/ROS2Bus.h>
#include <ROS2/Utilities/ROS2Names.h>

namespace ROS2
{
    void JointsTrajectoryComponent::Activate()
    {
        auto* ros2Frame = GetEntity()->FindComponent<ROS2FrameComponent>();
        AZ_Assert(ros2Frame, "Missing Frame Component!");
        AZStd::string namespacedAction = ROS2Names::GetNamespacedName(ros2Frame->GetNamespace(), m_followTrajectoryActionName);
        m_followTrajectoryServer = AZStd::make_unique<FollowJointTrajectoryActionServer>(namespacedAction, GetEntityId());
        AZ::TickBus::Handler::BusConnect();
        JointsTrajectoryRequestBus::Handler::BusConnect(GetEntityId());
    }

    ManipulationJoints& JointsTrajectoryComponent::GetManipulationJoints()
    {
        if (m_manipulationJoints.empty())
        {
            JointsManipulationRequestBus::EventResult(m_manipulationJoints, GetEntityId(), &JointsManipulationRequests::GetJoints);
        }
        return m_manipulationJoints;
    }

    void JointsTrajectoryComponent::Deactivate()
    {
        JointsTrajectoryRequestBus::Handler::BusDisconnect();
        AZ::TickBus::Handler::BusDisconnect();
        m_followTrajectoryServer.reset();
    }

    void JointsTrajectoryComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<JointsTrajectoryComponent, AZ::Component>()->Version(0)->Field(
                "Action name", &JointsTrajectoryComponent::m_followTrajectoryActionName);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<JointsTrajectoryComponent>("JointsTrajectoryComponent", "Component to control a robotic arm using trajectories")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->Attribute(AZ::Edit::Attributes::Icon, "Editor/Icons/Components/JointsTrajectoryComponent.svg")
                    ->Attribute(AZ::Edit::Attributes::ViewportIcon, "Editor/Icons/Components/Viewport/JointsTrajectoryComponent.svg")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &JointsTrajectoryComponent::m_followTrajectoryActionName,
                        "Action Name",
                        "Name the follow trajectory action server to accept movement commands");
            }
        }
    }

    void JointsTrajectoryComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("ROS2Frame"));
        required.push_back(AZ_CRC_CE("JointsManipulationService"));
    }

    void JointsTrajectoryComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("ManipulatorJointTrajectoryService"));
    }

    void JointsTrajectoryComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("ManipulatorJointTrajectoryService"));
    }

    AZ::Outcome<void, JointsTrajectoryComponent::TrajectoryResult> JointsTrajectoryComponent::StartTrajectoryGoal(
        TrajectoryGoalPtr trajectoryGoal)
    {
        if (m_trajectoryInProgress)
        {
            auto result = JointsTrajectoryComponent::TrajectoryResult();
            result.error_code = JointsTrajectoryComponent::TrajectoryResult::INVALID_GOAL;
            result.error_string = "Another trajectory goal is executing. Wait for completion or cancel it";
            return AZ::Failure(result);
        }

        auto validationResult = ValidateGoal(trajectoryGoal);
        if (!validationResult)
        {
            return validationResult;
        }
        m_trajectoryGoal = *trajectoryGoal;
        m_trajectoryExecutionStartTime = rclcpp::Time(ROS2::ROS2Interface::Get()->GetROSTimestamp());
        m_trajectoryInProgress = true;
        return AZ::Success();
    }

    AZ::Outcome<void, JointsTrajectoryComponent::TrajectoryResult> JointsTrajectoryComponent::ValidateGoal(TrajectoryGoalPtr trajectoryGoal)
    {
        // Check joint names validity
        for (const auto& jointName : trajectoryGoal->trajectory.joint_names)
        {
            AZStd::string azJointName(jointName.c_str());
            if (m_manipulationJoints.find(azJointName) == m_manipulationJoints.end())
            {
                AZ_Printf("JointsTrajectoryComponent", "Trajectory goal is invalid: no joint %s in manipulator", azJointName.c_str());

                auto result = JointsTrajectoryComponent::TrajectoryResult();
                result.error_code = JointsTrajectoryComponent::TrajectoryResult::INVALID_JOINTS;
                result.error_string = std::string(
                    AZStd::string::format("Trajectory goal is invalid: no joint %s in manipulator", azJointName.c_str()).c_str());

                return AZ::Failure(result);
            }
        }
        return AZ::Success();
    }

    void JointsTrajectoryComponent::UpdateFeedback()
    {
        auto goalStatus = GetGoalStatus();
        if (goalStatus != JointsTrajectoryRequests::TrajectoryActionStatus::Executing)
        {
            return;
        }

        auto feedback = std::make_shared<control_msgs::action::FollowJointTrajectory::Feedback>();

        trajectory_msgs::msg::JointTrajectoryPoint desiredPoint = m_trajectoryGoal.trajectory.points.front();

        trajectory_msgs::msg::JointTrajectoryPoint actualPoint;

        size_t jointCount = m_trajectoryGoal.trajectory.joint_names.size();
        for (size_t jointIndex = 0; jointIndex < jointCount; jointIndex++)
        {
            AZStd::string jointName(m_trajectoryGoal.trajectory.joint_names[jointIndex].c_str());
            std::string jointNameStdString(jointName.c_str());
            feedback->joint_names.push_back(jointNameStdString);

            float currentJointPosition;
            float currentJointVelocity;
            auto& jointInfo = m_manipulationJoints[jointName];
            PhysX::ArticulationJointRequestBus::Event(
                jointInfo.m_entityComponentIdPair.GetEntityId(),
                [&](PhysX::ArticulationJointRequests* articulationJointRequests)
                {
                    currentJointPosition = articulationJointRequests->GetJointPosition(jointInfo.m_axis);
                    currentJointVelocity = articulationJointRequests->GetJointVelocity(jointInfo.m_axis);
                });

            actualPoint.positions.push_back(static_cast<double>(currentJointPosition));
            actualPoint.velocities.push_back(static_cast<double>(currentJointVelocity));
            // Acceleration should also be filled in somehow, or removed from the trajectory altogether.
        }

        trajectory_msgs::msg::JointTrajectoryPoint currentError;
        for (size_t jointIndex = 0; jointIndex < jointCount; jointIndex++)
        {
            currentError.positions.push_back(actualPoint.positions[jointIndex] - desiredPoint.positions[jointIndex]);
            currentError.velocities.push_back(actualPoint.velocities[jointIndex] - desiredPoint.velocities[jointIndex]);
        }

        feedback->desired = desiredPoint;
        feedback->actual = actualPoint;
        feedback->error = currentError;

        m_followTrajectoryServer->PublishFeedback(feedback);
    }

    AZ::Outcome<void, AZStd::string> JointsTrajectoryComponent::CancelTrajectoryGoal()
    {
        m_trajectoryGoal.trajectory.points.clear();
        m_trajectoryInProgress = false;
        return AZ::Success();
    }

    JointsTrajectoryRequests::TrajectoryActionStatus JointsTrajectoryComponent::GetGoalStatus()
    {
        return m_followTrajectoryServer->GetGoalStatus();
    }

    void JointsTrajectoryComponent::FollowTrajectory(const uint64_t deltaTimeNs)
    {
        auto goalStatus = GetGoalStatus();
        if (goalStatus == JointsTrajectoryRequests::TrajectoryActionStatus::Cancelled)
        {
            JointsManipulationRequestBus::Event(GetEntityId(), &JointsManipulationRequests::Stop);
            auto result = std::make_shared<FollowJointTrajectoryActionServer::FollowJointTrajectory::Result>();
            result->error_string = "User Cancelled";
            result->error_code = FollowJointTrajectoryActionServer::FollowJointTrajectory::Result::SUCCESSFUL;
            m_followTrajectoryServer->CancelGoal(result);
            m_followTrajectoryServer->SetGoalSuccess();
            return;
        }

        if (goalStatus != JointsTrajectoryRequests::TrajectoryActionStatus::Executing)
        {
            return;
        }

        if (m_trajectoryGoal.trajectory.points.size() == 0)
        { // The manipulator has reached the goal.
            AZ_TracePrintf("JointsManipulationComponent", "Goal Concluded: all points reached\n");
            auto successResult = std::make_shared<control_msgs::action::FollowJointTrajectory::Result>(); //!< Empty defaults to success.
            m_followTrajectoryServer->GoalSuccess(successResult);
            m_trajectoryInProgress = false;
            return;
        }

        auto desiredGoal = m_trajectoryGoal.trajectory.points.front();
        rclcpp::Duration targetGoalTime = rclcpp::Duration(desiredGoal.time_from_start); //!< Requested arrival time for trajectory point.
        rclcpp::Time timeNow = rclcpp::Time(ROS2::ROS2Interface::Get()->GetROSTimestamp()); //!< Current simulation time.
        rclcpp::Duration threshold = rclcpp::Duration::from_nanoseconds(1e7);

        if (m_trajectoryExecutionStartTime + targetGoalTime <= timeNow + threshold)
        { // Jump to the next point if current simulation time is ahead of timeFromStart
            m_trajectoryGoal.trajectory.points.erase(m_trajectoryGoal.trajectory.points.begin());
            FollowTrajectory(deltaTimeNs);
            return;
        }

        MoveToNextPoint(desiredGoal);
    }

    void JointsTrajectoryComponent::MoveToNextPoint(const trajectory_msgs::msg::JointTrajectoryPoint currentTrajectoryPoint)
    {
        for (int jointIndex = 0; jointIndex < m_trajectoryGoal.trajectory.joint_names.size(); jointIndex++)
        { // Order each joint to be moved
            AZStd::string jointName(m_trajectoryGoal.trajectory.joint_names[jointIndex].c_str());
            AZ_Assert(m_manipulationJoints.find(jointName) != m_manipulationJoints.end(), "Invalid trajectory executing");

            float targetPos = currentTrajectoryPoint.positions[jointIndex];
            AZ::Outcome<void, AZStd::string> result;
            JointsManipulationRequestBus::EventResult(
                result, GetEntityId(), &JointsManipulationRequests::MoveJointToPosition, jointName, targetPos);
            AZ_Warning("JointTrajectoryComponent", result, "Joint move cannot be realized: %s", result.GetError().c_str());
        }
    }

    void JointsTrajectoryComponent::OnTick(float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
        if (m_manipulationJoints.empty())
        {
            GetManipulationJoints();
            return;
        }
        uint64_t deltaTimeNs = deltaTime * 1'000'000'000;
        FollowTrajectory(deltaTimeNs);
        UpdateFeedback();
    }
} // namespace ROS2
