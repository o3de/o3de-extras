#pragma once

#include <ROS2/Utilities/Controllers/PidConfiguration.h>

namespace ROS2
{
    //! Responsible for execution of command to move robotic arm (manipulator) with PID controller.
    class ManipulatorTrajectoryPIDController
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
        float ComputePIDJointVelocity(
            const float currentPosition, const float desiredPosition, const uint64_t& deltaTimeNs, int& jointIndex);
        void SetJointVelocity(const AZ::EntityComponentIdPair idPair, const float desiredVelocity);

        AZStd::vector<Controllers::PidConfiguration> m_pidConfigurationVector;
    };
} // namespace ROS2
