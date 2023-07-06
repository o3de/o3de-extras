/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root
 * of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Component/Component.h>
#include <AzCore/Component/TickBus.h>
#include <ROS2/Gripper/GripperRequestBus.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/gripper_command.hpp>

namespace ROS2
{
    class GripperActionServer
        : public AZ::Component
        , private AZ::TickBus::Handler
    {
    public:
        using GripperCommand = control_msgs::action::GripperCommand;
        using GoalHandleGripperCommand = rclcpp_action::ServerGoalHandle<control_msgs::action::GripperCommand>;
        AZ_COMPONENT(GripperActionServer, "{6A4417AC-1D85-4AB0-A116-1E77D40FC816}", AZ::Component);
        GripperActionServer() = default;
        ~GripperActionServer() = default;

        // AZ::Component overrides...
        void Activate() override;
        void Deactivate() override;

        static void Reflect(AZ::ReflectContext* context);

    private:
        // action server callbacks
        rclcpp_action::GoalResponse GoalReceivedCallback(const rclcpp_action::GoalUUID & uuid,
                                               std::shared_ptr<const GripperCommand::Goal> goal);

        rclcpp_action::CancelResponse GoalCancelledCallback(const std::shared_ptr<GoalHandleGripperCommand> goal_handle);

        void GoalAcceptedCallback(const std::shared_ptr<GoalHandleGripperCommand> goal_handle);

        // AZ::TickBus::Handler overrides ...
        void OnTick(float delta, AZ::ScriptTimePoint timePoint) override;

        //! Check if the goal is in an active state
        bool IsGoalActiveState() const;

        bool IsReadyForExecution() const;

        std::shared_ptr<GripperActionServer::GripperCommand::Feedback> ProduceFeedback() const;
        std::shared_ptr<GripperActionServer::GripperCommand::Result> ProduceResult() const;


        rclcpp_action::Server<GripperCommand>::SharedPtr actionServer;
        AZStd::string m_gripperActionServerName;

        std::shared_ptr<GoalHandleGripperCommand> m_goalHandle;
    };
} // namespace ROS2
