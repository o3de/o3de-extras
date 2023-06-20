#pragma once

#include <ROS2/Utilities/Controllers/PidConfiguration.h>

namespace ROS2
{
    //! Responsible for execution of command to move robotic arm (manipulator).
    class ManipulatorJointTrajectoryComponent
    {
    public:
        ManipulatorJointTrajectoryComponent() = default;
        virtual ~ManipulatorJointTrajectoryComponent() = default;

        enum class ControllerType
        {
            FeedForward, //!< @see <a href="https://en.wikipedia.org/wiki/Feed_forward_(control)">FeedForward</a>.
            PID, //!< @see <a href="https://en.wikipedia.org/wiki/PID_controller">PID</a>.
            PhysXArticulation, //!< PhysX Articulation
        };

        void SetTrajectoryGoal(const ManipulatorTrajectoryRequestBus::TrajectoryGoal& trajectoryGoal);

    private:
        virtual void KeepStillPosition(const uint64_t deltaTimeNs);
        virtual void FollowTrajectory(const uint64_t deltaTimeNs);

        ManipulatorTrajectoryRequestBus::TrajectoryGoal m_trajectoryGoal;
        AZStd::unordered_map<AZStd::string, float> m_initialPositions;
        rclcpp::Time m_trajectoryExecutionStartTime;

        bool m_initialized{ false };
        bool m_initializedTrajectory{ false };
    };
} // namespace ROS2
