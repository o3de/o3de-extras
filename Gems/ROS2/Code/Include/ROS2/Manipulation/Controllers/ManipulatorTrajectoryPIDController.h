#pragma once

#include <ROS2/Utilities/Controllers/PidConfiguration.h>

namespace ROS2
{
    //! Responsible for execution of command to move robotic arm (manipulator).
    class ManipulatorJointTrajectoryComponent
    {
    public:
        enum class ControllerType
        {
            FeedForward, //!< @see <a href="https://en.wikipedia.org/wiki/Feed_forward_(control)">FeedForward</a>.
            PID, //!< @see <a href="https://en.wikipedia.org/wiki/PID_controller">PID</a>.
            PhysXArticulation, //!< PhysX Articulation
        };

    private:
        void InitializePid();
        void InitializeCurrentPosition();
        void InitializePosition();
        void KeepStillPosition(const uint64_t deltaTimeNs);
        void FollowTrajectory(const uint64_t deltaTimeNs);

        float ComputeFFJointVelocity(const float currentPosition, const float desiredPosition, const rclcpp::Duration& duration) const;
        float ComputePIDJointVelocity(
            const float currentPosition, const float desiredPosition, const uint64_t& deltaTimeNs, int& jointIndex);
        void SetJointVelocity(const AZ::EntityComponentIdPair idPair, const float desiredVelocity);

        AZStd::vector<Controllers::PidConfiguration> m_pidConfigurationVector;
        AZStd::unordered_map<AZStd::string, float> m_initialPositions;
        AZStd::unordered_map<AZ::Name, float> m_jointKeepStillPosition;
        rclcpp::Time m_timeStartingExecutionTraj;

        float ComputeFFJointVelocity(const float currentPosition, const float desiredPosition, const rclcpp::Duration& duration) const;
        float ComputePIDJointVelocity(
            const float currentPosition, const float desiredPosition, const uint64_t& deltaTimeNs, int& jointIndex);
        void SetJointVelocity(const AZ::EntityComponentIdPair idPair, const float desiredVelocity);

        bool m_initialized{ false };
        bool m_initializedTrajectory{ false };
        bool m_setInitalPose{ false };
        bool m_keepStillPositionInitialize{ false };
    };
} // namespace ROS2
