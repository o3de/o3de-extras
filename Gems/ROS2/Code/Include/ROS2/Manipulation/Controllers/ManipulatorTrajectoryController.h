#pragma once

#include <ROS2/Manipulation/Controllers/ManipulatorTrajectoryRequestBus.h>
#include <ROS2/Manipulation/ManipulatorTrajectoryRequestBus.h>

namespace ROS2
{
    //! Base class responsible for execution of command to move robotic arm (manipulator) based on set trajectory goal.
    //! Can be derived to support different implementations.
    class ManipulatorJointTrajectoryComponent
    {
    public:
        ManipulatorJointTrajectoryComponent(const AZStd::string& actionName, const AZ::EntityId& entityId);
        virtual ~ManipulatorJointTrajectoryComponent() = default;

        enum class ControllerType
        {
            FeedForward, //!< @see <a href="https://en.wikipedia.org/wiki/Feed_forward_(control)">FeedForward</a>.
            PID, //!< @see <a href="https://en.wikipedia.org/wiki/PID_controller">PID</a>.
            PhysXArticulation, //!< PhysX Articulation
        };

        //! @see ROS2::ManipulatorTrajectoryRequestBus::StartTrajectoryGoal.
        void SetTrajectoryGoal(ManipulatorTrajectoryRequestBus::TrajectoryGoalPtr trajectoryGoal);

        //! @see ROS2::ManipulatorTrajectoryRequestBus::CancelCurrentGoal.
        void CancelCurrentGoal(ManipulatorTrajectoryRequestBus::TrajectoryResultPtr trajectoryResult);

        //! Follow set trajectory.
        //! This function is to be called with each time step after setting the trajectory with SetTrajectoryGoal.
        //! @param deltaTimeNs frame time step, to advance trajectory by.
        void FollowTrajectory(const uint64_t deltaTimeNs);

    protected:

        AZ::EntityId GetEntityId() const;


    private:
        virtual void MoveToNextPoint(
            const trajectory_msgs::msg::JointTrajectoryPoint currentTrajectoryPoint,
            uint64_t deltaTimeNs,
            const rclcpp::Time& simulationTimeNow) = 0;
        virtual void KeepStillPosition(const uint64_t deltaTimeNs) = 0;

        AZStd::string m_followTrajectoryActionName { "arm_controller/follow_joint_trajectory" };
        AZStd::unique_ptr<FollowJointTrajectoryActionServer> m_followTrajectoryServer;
        ManipulatorTrajectoryRequestBus::TrajectoryGoal m_trajectoryGoal;
        ManipulatorJoints m_manipulatorJoints;
        rclcpp::Time m_trajectoryExecutionStartTime;
        AZ::EntityId m_entityId;

        bool m_initialized{ false };
        bool m_trajectoryInProgress{ false };
    };
} // namespace ROS2
