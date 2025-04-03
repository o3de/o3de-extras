#pragma once

#include "Interfaces/IROS2HandlerBase.h"
#include "Utils/RegistryUtils.h"
#include <AzCore/std/functional.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <simulation_interfaces/msg/simulator_features.hpp>

namespace SimulationInterfacesROS2
{
    //! Base for each ROS2 action server, forces declaration of features provided by the server
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

        void CreateAction(rclcpp::Node::SharedPtr& node)
        {
            // Get the action name from the type name
            AZStd::string actionName = RegistryUtilities::GetName(GetTypeName());

            if (actionName.empty())
            {
                // if the action name is empty, use the default name
                actionName = GetDefaultName();
            }

            const std::string actionNameStr{ actionName.c_str(), actionName.size() };
            m_actionHandle = rclcpp_action::create_server<RosActionType>(
                node,
                actionNameStr,
                AZStd::bind(&ROS2ActionBase::GoalReceivedCallback, this, AZStd::placeholders::_1, AZStd::placeholders::_2),
                AZStd::bind(&ROS2ActionBase::GoalCancelledCallback, this, AZStd::placeholders::_1),
                AZStd::bind(&ROS2ActionBase::GoalAcceptedCallback, this, AZStd::placeholders::_1));
        }

    protected:
        //! This function is called when the action is cancelled
        virtual void CancelGoal(std::shared_ptr<Result> result)
        {
            AZ_Assert(m_goalHandle, "Invalid goal handle!");
            if (m_goalHandle && m_goalHandle->is_canceling())
            {
                m_goalHandle->canceled(result);
            }
        }

        //! This function is called when the action successfully finished
        virtual void GoalSuccess(std::shared_ptr<Result> result)
        {
            AZ_Assert(m_goalHandle, "Invalid goal handle!");
            if (m_goalHandle && (m_goalHandle->is_executing() || m_goalHandle->is_canceling()))
            {
                m_goalHandle->succeed(result);
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

        //! This function checks if the server is busy
        virtual bool IsGoalActiveState() const
        {
            if (!m_goalHandle)
            {
                return false;
            }
            return m_goalHandle->is_active() || m_goalHandle->is_executing() || m_goalHandle->is_canceling();
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
            return IsGoalActiveState() == false;
        }

        //! This function is called when the new goal receives
        virtual rclcpp_action::GoalResponse GoalReceivedCallback(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const Goal> goal) = 0;

        //! This function is called when the currently executed goal is cancelled
        virtual rclcpp_action::CancelResponse GoalCancelledCallback(const std::shared_ptr<GoalHandle> goal_handle) = 0;

        //! This function is called when the newly received goal is accepted
        virtual void GoalAcceptedCallback(const std::shared_ptr<GoalHandle> goal_handle) = 0;

        //! return features id defined by the handler, ids must follow the definition inside standard:
        //! @see https://github.com/ros-simulation/simulation_interfaces/blob/main/msg/SimulatorFeatures.msg
        AZStd::unordered_set<AZ::u8> GetProvidedFeatures() override
        {
            return {};
        };

        std::shared_ptr<GoalHandle> m_goalHandle;

    private:
        ActionHandle m_actionHandle;
    };
} // namespace SimulationInterfacesROS2
