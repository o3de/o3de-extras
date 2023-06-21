/*
* Copyright (c) Contributors to the Open 3D Engine Project.
* For complete copyright and license terms please see the LICENSE at the root of this distribution.
*
* SPDX-License-Identifier: Apache-2.0 OR MIT
*
 */

#pragma once

#include <AzCore/Component/Component.h>
#include <AzCore/Component/TickBus.h>
#include <AzCore/Entity/EntityBus.h>
#include <ROS2/Manipulation/JointsTrajectoryRequestBus.h>
#include <control_msgs/action/follow_joint_trajectory.hpp>

namespace ROS2
{
    //! Component responsible for execution of commands to move robotic arm (manipulator) based on set trajectory goal.
    class JointsTrajectoryComponent
        : public AZ::Component
        , public AZ::TickBus::Handler
        , public JointsTrajectoryRequestBus::Handler
    {
    public:
        JointsTrajectoryComponent() = default;
        ~JointsTrajectoryComponent() = default;
        AZ_COMPONENT(JointsTrajectoryComponent, "{429DE04C-6B6D-4B2D-9D6C-3681F23CBF90}", AZ::Component);

        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void Reflect(AZ::ReflectContext* context);

        // JointsTrajectoryRequestBus::Handler overrides ...
        //! @see ROS2::JointsTrajectoryRequestBus::StartTrajectoryGoal
        AZ::Outcome<void, AZStd::string> StartTrajectoryGoal(TrajectoryGoalPtr trajectoryGoal) override;
        //! @see ROS2::JointsTrajectoryRequestBus::CancelTrajectoryGoal
        AZ::Outcome<void, AZStd::string> CancelTrajectoryGoal(TrajectoryResultPtr trajectoryResult) override;
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
        AZ::Outcome<void, AZStd::string> ValidateGoal(TrajectoryGoalPtr trajectoryGoal);
        void MoveToNextPoint(const trajectory_msgs::msg::JointTrajectoryPoint currentTrajectoryPoint);
        void UpdateFeedback();

        AZStd::string m_followTrajectoryActionName{ "arm_controller/follow_joint_trajectory" };
        AZStd::unique_ptr<FollowJointTrajectoryActionServer> m_followTrajectoryServer;
        JointsTrajectoryRequestBus::TrajectoryGoal m_trajectoryGoal;
        ManipulationJoints m_manipulationJoints;
        rclcpp::Time m_trajectoryExecutionStartTime;

        bool m_trajectoryInProgress{ false };
    };
} // namespace ROS2