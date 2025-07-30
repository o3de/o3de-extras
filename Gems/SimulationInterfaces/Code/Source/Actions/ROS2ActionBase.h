/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/std/functional.h>
#include <Interfaces/IROS2HandlerBase.h>
#include <Utils/RegistryUtils.h>
#include <rclcpp_action/rclcpp_action.hpp>
#include <simulation_interfaces/msg/simulator_features.hpp>

namespace ROS2SimulationInterfaces
{
    //! Base for each ROS 2 action server handler, forces declaration of features provided by the server
    //! combined information along all ROS 2 handlers gives information about simulation features
    //! @see https://github.com/ros-simulation/simulation_interfaces/blob/main/msg/SimulatorFeatures.msg
    using SimulationFeatures = simulation_interfaces::msg::SimulatorFeatures;
    template<typename RosActionType>
    class ROS2ActionBase : public virtual IROS2HandlerBase
    {
    public:
        using Result = typename RosActionType::Result;
        using Feedback = typename RosActionType::Feedback;
        using Goal = typename RosActionType::Goal;
        using ActionHandle = typename rclcpp_action::Server<RosActionType>::SharedPtr;
        using GoalHandle = rclcpp_action::ServerGoalHandle<RosActionType>;

        virtual ~ROS2ActionBase() = default;

        void Initialize(rclcpp::Node::SharedPtr& node) override
        {
            CreateAction(node);
        }

        bool IsValid() override
        {
            return m_actionHandle != nullptr;
        }

    protected:
        //! This function is called when the action is cancelled
        virtual void CancelGoal(std::shared_ptr<Result> result)
        {
            AZ_Assert(m_goalHandle, "Invalid goal handle!");
            if (m_goalHandle && m_goalHandle->is_canceling())
            {
                m_goalHandle->canceled(result);
                m_goalHandle.reset();
            }
        }

        //! This function is called when the action successfully finished
        virtual void GoalSuccess(std::shared_ptr<Result> result)
        {
            AZ_Assert(m_goalHandle, "Invalid goal handle!");
            if (m_goalHandle && (m_goalHandle->is_executing() || m_goalHandle->is_canceling()))
            {
                m_goalHandle->succeed(result);
                m_goalHandle.reset();
            }
        }

        //! This function is called on demand, based on the action server implementation, to share the action progress
        virtual void PublishFeedback(std::shared_ptr<Feedback> feedback)
        {
            AZ_Assert(m_goalHandle, "Invalid goal handle!");
            if (m_goalHandle && m_goalHandle->is_executing())
            {
                m_goalHandle->publish_feedback(feedback);
            }
        }

        //! This function check if the server is ready to handle new goal
        virtual bool IsReadyForExecution() const
        {
            // Has no goal handle yet - can be accepted.
            if (!m_goalHandle)
            {
                return true;
            }
            // Accept if the previous one is in a terminal state.
            return !(m_goalHandle->is_active() || m_goalHandle->is_executing() || m_goalHandle->is_canceling());
        }

        //! This function is called when the new goal receives
        virtual rclcpp_action::GoalResponse GoalReceivedCallback(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const Goal> goal) = 0;

        //! This function is called when the currently executed goal is cancelled
        virtual rclcpp_action::CancelResponse GoalCancelledCallback(const std::shared_ptr<GoalHandle> goal_handle) = 0;

        //! This function is called when the newly received goal is accepted
        virtual void GoalAcceptedCallback(const std::shared_ptr<GoalHandle> goal_handle) = 0;

        //! return features id defined by the handler, ids must follow the definition inside standard:
        //! @see https://github.com/ros-simulation/simulation_interfaces/blob/main/msg/SimulatorFeatures.msg
        AZStd::unordered_set<SimulationFeatureType> GetProvidedFeatures() override
        {
            return {};
        };

        std::shared_ptr<GoalHandle> m_goalHandle;

    private:
        void CreateAction(rclcpp::Node::SharedPtr& node)
        {
            // Get the action name from the type name
            // passing empty string to settings registry causes in not creating ROS 2 action
            AZStd::optional<AZStd::string> actionName = RegistryUtilities::GetName(GetTypeName());

            // check if settings registry and apply early exit if value is equal to empty string
            if (actionName.has_value())
            {
                if (actionName.value().empty())
                {
                    AZ_Warning(
                        "SimulationInterfaces",
                        false,
                        "Action name for type %s is set to empty string, action server won't be created",
                        GetTypeName().data());
                    return;
                }
            }

            if (!actionName.has_value())
            {
                // if the service name is empty, use the default name
                actionName = GetDefaultName();
            }

            const std::string actionNameStr{ actionName.value().c_str(), actionName.value().size() };
            m_actionHandle = rclcpp_action::create_server<RosActionType>(
                node,
                actionNameStr,
                AZStd::bind(&ROS2ActionBase::GoalReceivedCallback, this, AZStd::placeholders::_1, AZStd::placeholders::_2),
                AZStd::bind(&ROS2ActionBase::GoalCancelledCallback, this, AZStd::placeholders::_1),
                AZStd::bind(&ROS2ActionBase::GoalAcceptedCallback, this, AZStd::placeholders::_1));
        }

        ActionHandle m_actionHandle;
    };
} // namespace ROS2SimulationInterfaces
