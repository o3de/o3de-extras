/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include "ROS2ActionBase.h"
#include <AzCore/Component/TickBus.h>
#include <SimulationInterfaces/SimulationMangerRequestBus.h>
#include <simulation_interfaces/action/simulate_steps.hpp>

namespace ROS2SimulationInterfaces
{

    class SimulateStepsActionServerHandler
        : public ROS2ActionBase<simulation_interfaces::action::SimulateSteps>
        , public AZ::TickBus::Handler
        , public SimulationInterfaces::SimulationManagerNotificationsBus::Handler
    {
    public:
        ~SimulateStepsActionServerHandler();

        // IROS2HandlerBase overrides
        AZStd::unordered_set<SimulationFeatureType> GetProvidedFeatures() override;
        AZStd::string_view GetTypeName() const override;
        AZStd::string_view GetDefaultName() const override;
        void Initialize(rclcpp::Node::SharedPtr& node) override;

        // ROS2ActionBase<simulation_interfaces::action::SimulateSteps> overrides
        rclcpp_action::GoalResponse GoalReceivedCallback(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const Goal> goal) override;
        rclcpp_action::CancelResponse GoalCancelledCallback(const std::shared_ptr<GoalHandle> goal_handle) override;
        void GoalAcceptedCallback(const std::shared_ptr<GoalHandle> goal_handle) override;

    protected:
        // AZ::TickBus::Handler overrides
        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;

        // SimulationInterfaces::SimulationManagerNotificationsBus::Handler
        void OnSimulationStepFinish(const AZ::u64 remainingSteps) override;

    private:
        AZ::u64 m_goalSteps{ 0 };
        bool m_isCancelled{ false };
    };

} // namespace ROS2SimulationInterfaces
