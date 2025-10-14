/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include "FollowJointTrajectoryActionServer.h"
#include <AzCore/Component/Component.h>
#include <AzCore/Component/EntityBus.h>
#include <AzCore/Component/TickBus.h>
#include <ROS2Controllers/Manipulation/JointsManipulationRequests.h>
#include <ROS2Controllers/Manipulation/JointsTrajectoryRequests.h>
#include <ROS2Controllers/ROS2ControllersTypeIds.h>
#include <control_msgs/action/follow_joint_trajectory.hpp>

namespace ROS2Controllers
{
    //! Component responsible for execution of commands to move robotic arm (manipulator) based on set trajectory goal.
    class JointsTrajectoryComponent
        : public AZ::Component
        , public AZ::TickBus::Handler
        , public JointsTrajectoryRequestBus::Handler
    {
    public:
        JointsTrajectoryComponent() = default;
        JointsTrajectoryComponent(const AZStd::string& followTrajectoryActionName);
        ~JointsTrajectoryComponent() = default;
        AZ_COMPONENT(JointsTrajectoryComponent, JointsTrajectoryComponentTypeId, AZ::Component);

        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void Reflect(AZ::ReflectContext* context);

        // JointsTrajectoryRequestBus::Handler overrides ...
        //! @see ROS2::JointsTrajectoryRequestBus::StartTrajectoryGoal
        AZ::Outcome<void, TrajectoryResult> StartTrajectoryGoal(TrajectoryGoalPtr trajectoryGoal) override;
        //! @see ROS2::JointsTrajectoryRequestBus::CancelTrajectoryGoal
        AZ::Outcome<void, AZStd::string> CancelTrajectoryGoal() override;
        //! @see ROS2::JointsTrajectoryRequestBus::GetGoalStatus
        TrajectoryActionStatus GetGoalStatus() override;

    private:
        // Component overrides ...
        void Activate() override;
        void Deactivate() override;

        // AZ::TickBus::Handler overrides
        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;

        //! Follow set trajectory.
        //! @param deltaTimeNs frame time step, to advance trajectory by.
        void FollowTrajectory(const uint64_t deltaTimeNs);
        AZ::Outcome<void, TrajectoryResult> ValidateGoal(TrajectoryGoalPtr trajectoryGoal);
        bool CheckIfPositionReachedTolerance(const trajectory_msgs::msg::JointTrajectoryPoint currentTrajectoryPoint);
        bool CheckIfVelocityReachedTolerance();
        void MoveToNextPoint(const trajectory_msgs::msg::JointTrajectoryPoint currentTrajectoryPoint);
        void UpdateFeedback();

        //! Lazy initialize Manipulation joints on the start of simulation.
        ManipulationJoints& GetManipulationJoints();

        AZStd::string m_followTrajectoryActionName{ "arm_controller/follow_joint_trajectory" };
        AZStd::unique_ptr<FollowJointTrajectoryActionServer> m_followTrajectoryServer;
        TrajectoryGoal m_trajectoryGoal;
        TrajectoryActionStatus m_goalStatus = TrajectoryActionStatus::Idle;
        rclcpp::Time m_trajectoryExecutionStartTime;
        ManipulationJoints m_manipulationJoints;
        builtin_interfaces::msg::Time m_lastTickTimestamp; //!< ROS 2 Timestamp during last OnTick call

        bool m_checkForPositionErrors = false; //!< If true, check if joints reached the goal position before reporting success
        float m_jointPositionTolerance = 0.04f; //!< The threshold for joint position errors to report the goal as reached
        bool ShouldCheckForPositionErrors();

        bool m_checkForVelocity = false; //!< If true, check if joints velocity is below threshold before reporting success
        float m_jointVelocityTolerance = 0.01f; //!< The threshold for joint velocities under which to report the goal as reached
        bool ShouldCheckForVelocity();
    };
} // namespace ROS2Controllers
