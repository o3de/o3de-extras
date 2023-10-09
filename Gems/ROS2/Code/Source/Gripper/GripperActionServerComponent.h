/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root
 * of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include "GripperActionServer.h"
#include <AzCore/Component/Component.h>
#include <AzCore/Component/TickBus.h>
#include <ROS2/Gripper/GripperRequestBus.h>
#include <control_msgs/action/gripper_command.hpp>
#include <rclcpp/rclcpp.hpp>

namespace ROS2
{
    //! GripperActionServerComponent is a component that encapsulates gripper action server.
    //! It is responsible for creating and managing the action server, producing feedback and result.
    class GripperActionServerComponent
        : public AZ::Component
        , private AZ::TickBus::Handler
    {
    public:
        using GripperCommand = control_msgs::action::GripperCommand;
        using GoalHandleGripperCommand = rclcpp_action::ServerGoalHandle<control_msgs::action::GripperCommand>;
        AZ_COMPONENT(GripperActionServerComponent, "{6A4417AC-1D85-4AB0-A116-1E77D40FC816}", AZ::Component);
        GripperActionServerComponent() = default;
        ~GripperActionServerComponent() = default;
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void Reflect(AZ::ReflectContext* context);

    private:
        // AZ::Component overrides...
        void Activate() override;
        void Deactivate() override;

        // AZ::TickBus::Handler overrides
        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;

        std::shared_ptr<GripperActionServer::GripperCommand::Feedback> ProduceFeedback() const;
        std::shared_ptr<GripperActionServer::GripperCommand::Result> ProduceResult() const;
        AZStd::string m_gripperActionServerName{ "gripper_server" }; //! name of the GripperCommand action server
        AZStd::unique_ptr<GripperActionServer> m_gripperActionServer; //! action server for GripperCommand
    };
} // namespace ROS2
