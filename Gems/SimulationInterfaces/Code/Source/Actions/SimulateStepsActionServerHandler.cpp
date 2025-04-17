/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "SimulateStepsActionServerHandler.h"

namespace ROS2SimulationInterfaces
{

    void SimulateStepsActionServerHandler::Initialize(rclcpp::Node::SharedPtr& node)
    {
        ROS2ActionBase::Initialize(node);
        SimulationInterfaces::SimulationManagerNotificationsBus::Handler::BusConnect();
    }

    SimulateStepsActionServerHandler::~SimulateStepsActionServerHandler()
    {
        AZ::TickBus::Handler::BusDisconnect();
        SimulationInterfaces::SimulationManagerNotificationsBus::Handler::BusDisconnect();
    }

    AZStd::unordered_set<SimulationFeatureType> SimulateStepsActionServerHandler::GetProvidedFeatures()
    {
        return AZStd::unordered_set<SimulationFeatureType>{ SimulationFeatures::STEP_SIMULATION_ACTION,
                                                            SimulationFeatures::STEP_SIMULATION_SINGLE,
                                                            SimulationFeatures::STEP_SIMULATION_MULTIPLE };
    }

    AZStd::string_view SimulateStepsActionServerHandler::GetTypeName() const
    {
        return "SimulateSteps";
    }

    AZStd::string_view SimulateStepsActionServerHandler::GetDefaultName() const
    {
        return "simulate_steps";
    }

    rclcpp_action::GoalResponse SimulateStepsActionServerHandler::GoalReceivedCallback(
        [[maybe_unused]] const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const Goal> goal)
    {
        if (!IsReadyForExecution())
        {
            return rclcpp_action::GoalResponse::REJECT;
        }

        m_goalSteps = goal->steps;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse SimulateStepsActionServerHandler::GoalCancelledCallback(
        [[maybe_unused]] const std::shared_ptr<GoalHandle> goal_handle)
    {
        SimulationInterfaces::SimulationManagerRequestBus::Broadcast(
            &SimulationInterfaces::SimulationManagerRequests::CancelStepSimulation);
        m_isCancelled = true;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void SimulateStepsActionServerHandler::GoalAcceptedCallback(const std::shared_ptr<GoalHandle> goal_handle)
    {
        m_goalHandle = goal_handle;

        bool isPaused = false;
        SimulationInterfaces::SimulationManagerRequestBus::BroadcastResult(
            isPaused, &SimulationInterfaces::SimulationManagerRequests::IsSimulationPaused);
        if (!isPaused)
        {
            //! If simulation is not paused then action should not be processed and action server should return Result with result code set
            //! to the RESULT_OPERATION_FAILED value, according to the documentation:
            //! https://github.com/ros-simulation/simulation_interfaces/blob/main/action/SimulateSteps.action
            auto result = std::make_shared<Result>();
            result->result.result = simulation_interfaces::msg::Result::RESULT_OPERATION_FAILED;
            result->result.error_message = "Request cannot be processed - simulation has to be paused. Action will be aborted.";
            m_goalHandle->abort(result);
            m_goalHandle.reset();
            return;
        }
        m_isCancelled = false;
        AZ::TickBus::Handler::BusConnect();
        SimulationInterfaces::SimulationManagerRequestBus::Broadcast(
            &SimulationInterfaces::SimulationManagerRequests::StepSimulation, m_goalSteps);
    }

    void SimulateStepsActionServerHandler::OnTick([[maybe_unused]] float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
        if (m_isCancelled)
        {
            auto result = std::make_shared<Result>();
            result->result.error_message = "Action has been cancelled.";
            result->result.result = simulation_interfaces::msg::Result::RESULT_OK;
            CancelGoal(result);
            AZ::TickBus::Handler::BusDisconnect();
            return;
        }

        // If SimulationSteps is active then it means that SimulationManagerRequestBus::Handler is busy now with a previous request
        bool isActive = true;
        SimulationInterfaces::SimulationManagerRequestBus::BroadcastResult(
            isActive, &SimulationInterfaces::SimulationManagerRequests::IsSimulationStepsActive);
        if (!isActive)
        {
            auto result = std::make_shared<Result>();
            result->result.error_message = "Action finished.";
            result->result.result = simulation_interfaces::msg::Result::RESULT_OK;
            GoalSuccess(result);
            AZ::TickBus::Handler::BusDisconnect();
        }
    }

    void SimulateStepsActionServerHandler::OnSimulationStepFinish(const AZ::u64 remainingSteps)
    {
        auto feedback = std::make_shared<Feedback>();
        feedback->remaining_steps = remainingSteps;
        feedback->completed_steps = m_goalSteps - remainingSteps;
        PublishFeedback(feedback);
    }

} // namespace ROS2SimulationInterfaces
