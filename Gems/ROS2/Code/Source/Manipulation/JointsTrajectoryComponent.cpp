/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "JointsTrajectoryComponent.h"
#include <AzCore/Serialization/EditContext.h>
#include <ROS2/Frame/ROS2FrameComponent.h>
#include <ROS2/Manipulation/JointsManipulationRequests.h>
#include <ROS2/ROS2Bus.h>
#include <ROS2/Utilities/ROS2Names.h>

namespace ROS2
{
    void JointsTrajectoryComponent::Activate()
    {
        auto ros2Node = ROS2::ROS2Interface::Get()->GetNode();
        auto* ros2Frame = Utils::GetGameOrEditorComponent<ROS2FrameComponent>(GetEntity());
        AZ_Assert(ros2Frame, "Missing Frame Component!");
        AZStd::string namespacedAction = ROS2Names::GetNamespacedName(ros2Frame->GetNamespace(), m_followTrajectoryActionName);
        m_followTrajectoryServer = AZStd::make_unique<FollowJointTrajectoryActionServer>(namespacedAction, GetEntityId());
        JointsManipulationRequestBus::EventResult(m_manipulationJoints, GetEntityId(), &JointsManipulationRequests::GetJoints);
        AZ::TickBus::Handler::BusConnect();
        JointsTrajectoryRequestBus::Handler::BusConnect(GetEntityId());
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

    AZ::Outcome<void, AZStd::string> JointsTrajectoryComponent::StartTrajectoryGoal(TrajectoryGoalPtr trajectoryGoal)
    {
        if (m_trajectoryInProgress)
        {
            return AZ::Failure("Another trajectory goal is executing. Wait for completion or cancel it");
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

    AZ::Outcome<void, AZStd::string> JointsTrajectoryComponent::ValidateGoal(TrajectoryGoalPtr trajectoryGoal)
    {
        // Check joint names validity
        for (const auto& jointName : trajectoryGoal->trajectory.joint_names)
        {
            AZStd::string azJointName(jointName.c_str());
            if (m_manipulationJoints.find(azJointName) == m_manipulationJoints.end())
            {
                AZ_Printf(
                    "JointsTrajectoryComponent",
                    "Trajectory goal is invalid: no joint %s in manipulator",
                    azJointName.c_str());
                // TODO - pass as a result, use FollowTrajectoryAction::Result enum
                return AZ::Failure(AZStd::string::format("Trajectory goal is invalid: no joint %s in manipulator", azJointName.c_str()));
            }
        }
        // TODO - other checks?
        return AZ::Success();
    }

    void JointsTrajectoryComponent::UpdateFeedback()
    {
        auto feedback = std::make_shared<control_msgs::action::FollowJointTrajectory::Feedback>();
        trajectory_msgs::msg::JointTrajectoryPoint desiredPoint;
        for (const auto& [jointName, hingeComponent] : m_manipulationJoints)
        {
            std::string jointNameStdString(jointName.c_str());
            feedback->joint_names.push_back(jointNameStdString);

            AZ::Outcome<float, AZStd::string> result;
            JointsManipulationRequestBus::EventResult(result, GetEntityId(), &JointsManipulationRequests::GetJointPosition, jointName);
            auto currentJointPosition = result.GetValue();

            desiredPoint.positions.push_back(static_cast<double>(currentJointPosition));
            desiredPoint.velocities.push_back(0.0f); // Velocities not supported yet!
            desiredPoint.accelerations.push_back(0.0f); // Accelerations not supported yet!
            desiredPoint.effort.push_back(0.0f); // Effort not supported yet!
        }

        // TODO - construct proper feedback
        trajectory_msgs::msg::JointTrajectoryPoint actualPoint = desiredPoint;
        trajectory_msgs::msg::JointTrajectoryPoint currentError;

        std::transform(
            desiredPoint.positions.begin(),
            desiredPoint.positions.end(),
            actualPoint.positions.begin(),
            currentError.positions.begin(),
            std::minus<double>());

        std::transform(
            desiredPoint.velocities.begin(),
            desiredPoint.velocities.end(),
            actualPoint.velocities.begin(),
            currentError.velocities.begin(),
            std::minus<double>());

        std::transform(
            desiredPoint.accelerations.begin(),
            desiredPoint.accelerations.end(),
            actualPoint.accelerations.begin(),
            currentError.accelerations.begin(),
            std::minus<double>());

        std::transform(
            desiredPoint.effort.begin(),
            desiredPoint.effort.end(),
            actualPoint.effort.begin(),
            currentError.effort.begin(),
            std::minus<double>());

        feedback->desired = desiredPoint;
        feedback->actual = actualPoint;
        feedback->error = currentError;
        m_followTrajectoryServer->PublishFeedback(feedback);
    }

    AZ::Outcome<void, AZStd::string> JointsTrajectoryComponent::CancelTrajectoryGoal(TrajectoryResultPtr result)
    {
        m_trajectoryGoal.trajectory.points.clear(); // TODO - empty the trajectory
        m_followTrajectoryServer->CancelGoal(result);
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
            return;
        }
        uint64_t deltaTimeNs = deltaTime * 1'000'000'000;
        FollowTrajectory(deltaTimeNs);
    }
} // namespace ROS2
