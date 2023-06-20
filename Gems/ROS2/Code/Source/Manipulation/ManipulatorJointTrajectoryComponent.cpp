/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <ROS2/Frame/ROS2FrameComponent.h>
#include <ROS2/Manipulation/ManipulatorJointTrajectoryComponent.h>
#include <ROS2/ROS2Bus.h>

namespace ROS2
{
    void ManipulatorJointTrajectoryComponent::Activate()
    {
        auto ros2Node = ROS2::ROS2Interface::Get()->GetNode();
        auto ros2Frame = GetEntityId()->FindComponent<ROS2FrameComponent>();
        AZ_Assert(ros2Frame, "Missing Frame Component!");
        AZStd::string namespacedAction = ROS2Names::GetNamespacedName(ros2Frame->GetNamespace(), actionName);
        m_followTrajectoryServer = AZStd::make_unique<FollowJointTrajectoryActionServer>(namespacedAction);

        AZ::TickBus::Handler::BusConnect();
        ManipulatorTrajectoryRequestBus::Handler::BusConnect(GetEntityId());
    }

    void ManipulatorJointTrajectoryComponent::Deactivate()
    {
        ManipulatorTrajectoryRequestBus::Handler::BusDisconnect();
        AZ::TickBus::Handler::BusDisconnect();
        m_followTrajectoryServer->reset();
    }

    void ManipulatorJointTrajectoryComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<ManipulatorJointTrajectoryComponent, AZ::Component>()->Version(0)->Field(
                "Action name", &ManipulatorComponent::m_followTrajectoryActionName);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<ManipulatorComponent>(
                      "ManipulatorJointTrajectoryComponent", "Component to control a robotic arm using trajectories")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ManipulatorComponent::m_followTrajectoryActionName,
                        "Action Name",
                        "Name the follow trajectory action server to accept movement commands")
            }
        }
    }

    void ManipulatorJointTrajectoryComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("ROS2Frame"));
        required.push_back(AZ_CRC_CE("ManipulatorService"));
    }

    void ManipulatorJointTrajectoryComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("ManipulatorJointTrajectoryService"));
    }

    AZ::Outcome<void, AZStd::string> ManipulatorJointTrajectoryComponent::StartTrajectoryGoal(TrajectoryGoalPtr trajectoryGoal)
    {
        if (m_trajectoryGoalInProgress)
        {
            return AZ::Failure("Another trajectory goal is executing. Wait for completion or cancel it");
        }

        auto validationResult = ValidateGoal(trajectoryGoal);
        if (!validationResult)
        {
            return validationResult;
        }
        m_trajectoryGoal = trajectoryGoal;
        m_trajectoryExecutionStartTime = rclcpp::Time(ROS2::ROS2Interface::Get()->GetROSTimestamp());
        m_trajectoryInProgress = true;
        return AZ::Success();
    }

    AZ::Outcome<void, AZStd::string> ManipulatorJointTrajectoryComponent::ValidateGoal(TrajectoryGoalPtr trajectoryGoal)
    {
        // Check joint names validity
        ManipulatorRequestBus::ManipulatorJoints manipulatorJoints;
        ManipulatorBus::EventResult(manipulatorJoints, GetEntityId(), &ManipulatorRequests::GetManipulatorJoints);
        for (const auto& jointName : trajectoryGoal->trajectory.joint_names)
        {
            auto azJointName = AZ::Name(jointName.c_str());
            if (manipulatorJoints.find(azJointName) == manipulatorJoints.end())
            {
                // TODO - pass as a result, use FollowTrajectoryAction::Result enum
                return AZ::Failure("Trajectory goal is invalid: no joint %s in manipulator", azJointName.GetCStr())
            }
        }
        // TODO - other checks?
        return AZ::Success();
    }

    void ManipulatorJointTrajectoryComponent::UpdateFeedback()
    {
        ManipulatorRequestBus::ManipulatorJoints manipulatorJoints;
        ManipulatorBus::EventResult(manipulatorJoints, GetEntityId(), &ManipulatorRequests::GetManipulatorJoints);
        auto feedback = std::make_shared(control_msgs::action::FollowJointTrajectory::Feedback);
        trajectory_msgs::msg::JointTrajectoryPoint desiredPoint;
        for (const auto& [jointName, hingeComponent] : manipulatorJoints)
        {
            std::string jointNameStdString(jointName.GetCStr());
            feedback->joint_names.push_back(jointNameStdString);

            AZ::Outcome<float, AZStd::string> result;
            ManipulatorBus::EventResult(result, GetEntityId(), &ManipulatorRequests::GetSingleDOFJointPosition, jointName);
            auto currentJointPosition = result.value();

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
            std::minus<double>);

        std::transform(
            desiredPoint.velocities.begin(),
            desiredPoint.velocities.end(),
            actualPoint.velocities.begin(),
            currentError.velocities.begin(),
            std::minus<double>);

        std::transform(
            desiredPoint.accelerations.begin(),
            desiredPoint.accelerations.end(),
            actualPoint.accelerations.begin(),
            currentError.accelerations.begin(),
            std::minus<double>);

        std::transform(
            desiredPoint.effort.begin(),
            desiredPoint.effort.end(),
            actualPoint.effort.begin(),
            currentError.effort.begin(),
            std::minus<double>);

        feedback->desired = desiredPoint;
        feedback->actual = actualPoint;
        feedback->error = currentError;
        feedback->duration = rclcpp::Time(ROS2::ROS2Interface::Get()->GetROSTimestamp()) - m_trajectoryExecutionStartTime;
        m_followTrajectoryServer->PublishFeedback(feedback);
    }

    AZ::Outcome<void, AZStd::string> ManipulatorJointTrajectoryComponent::CancelTrajectoryGoal(TrajectoryResultPtr result)
    {
        m_trajectory.trajectory.points.clear() // TODO - empty the trajectory
        m_followTrajectoryServer->CancelGoal(result);
        m_trajectoryInProgress = false;
        return AZ::Success();
    }

    GoalStatus ManipulatorComponent::GetGoalStatus()
    {
        return m_followTrajectoryServer->GetGoalStatus();
    }

    void ManipulatorJointTrajectoryComponent::FollowTrajectory(const uint64_t deltaTimeNs)
    {
        auto goalStatus = GetGoalStatus();
        if (goalStatus == GoalStatus::Cancelled)
        {
            ManipulatorRequestBus::Event(GetEntityId(), &ManipulatorRequests::Stop);
            return;
        }

        if (goalStatus != GoalStatus::Executing)
        {
            return;
        }

        if (m_trajectoryGoal.trajectory.points.size() == 0)
        { // The manipulator has reached the goal.
            AZ_TracePrintf("ManipulatorComponent", "Goal Concluded: all points reached");
            auto successResult = std::make_shared<control_msgs::action::FollowJointTrajectory::Result>(); //!< Empty defaults to success.
            m_followTrajectoryServer->GoalSuccess(successResult);
            m_trajectoryInProgress = false;
            return;
        }

        auto desiredGoal = trajectory.points.front();
        rclcpp::Duration targetGoalTime = rclcpp::Duration(desiredGoal.time_from_start); //!< Requested arrival time for trajectory point.
        rclcpp::Time timeNow = rclcpp::Time(ROS2::ROS2Interface::Get()->GetROSTimestamp()); //!< Current simulation time.
        rclcpp::Duration threshold = rclcpp::Duration::from_nanoseconds(1e7);

        if (m_trajectoryExecutionStartTime + targetGoalTime <= timeNow + threshold)
        { // Jump to the next point if current simulation time is ahead of timeFromStart
            m_trajectory.trajectory.points.erase(m_trajectory.trajectory.points.begin());
            FollowTrajectory(deltaTimeNs);
            return;
        }

        MoveToNextPoint(desiredGoal);
    }

    void ManipulatorJointTrajectoryComponent::MoveToNextPoint(const trajectory_msgs::msg::JointTrajectoryPoint currentTrajectoryPoint)
    {
        ManipulatorRequestBus::ManipulatorJoints manipulatorJoints;
        ManipulatorBus::EventResult(manipulatorJoints, GetEntityId(), &ManipulatorRequests::GetManipulatorJoints);
        for (int jointIndex = 0; jointIndex < m_trajectory.joint_names.size(); jointIndex++)
        { // Order each joint to be moved
            const auto& jointName = AZ::Name(m_trajectory.joint_names[jointIndex].c_str());
            AZ_Assert(manipulatorJoints.find(jointName) != manipulatorJoints.end(), "Invalid trajectory executing");

            float targetPos = nextTrajectoryPoint.positions[jointIndex];
            AZ::Outcome<void, AZStd::string> result;
            ManipulatorBus::EventResult(result, GetEntityId(), &ManipulatorRequests::MoveSingleDOFJointToPosition, jointName, targetPos);
            AZ_Assert(result, "Joint move cannot be realized: %s", result.error())
        }
    }
} // namespace ROS2
