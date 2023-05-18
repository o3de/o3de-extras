#pragma once

#include <AzCore/Component/Component.h>
#include <AzCore/Component/TickBus.h>
#include <AzCore/Name/Name.h>
#include <ROS2/Utilities/Controllers/PidConfiguration.h>
#include <control_msgs/action/follow_joint_trajectory.hpp>


namespace ROS2
{
    // forward declaration
    class FollowJointTrajectoryActionServer;

    //! Component responsible for controlling a robotic arm made up of hinge joints.
    class ManipulatorControllerComponent
        : public AZ::Component
        , public AZ::TickBus::Handler
    {
    public:
        enum class Controller
        {
            FeedForward, //!< @see <a href="https://en.wikipedia.org/wiki/Feed_forward_(control)">FeedForward</a>.
            PID,          //!< @see <a href="https://en.wikipedia.org/wiki/PID_controller">PID</a>.
            PhysXArticulation, //!< PhysX Articulation
        };

        AZ_COMPONENT(ManipulatorControllerComponent, "{3da9abfc-0028-4e3e-8d04-4e4440d2e319}", AZ::Component); // , ManipulatorRequestBus::Handler);

        ManipulatorControllerComponent();
        ~ManipulatorControllerComponent();

        //////////////////////////////////////////////////////////////////////////
        // Component overrides
        void Activate() override;
        void Deactivate() override;
        //////////////////////////////////////////////////////////////////////////
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void Reflect(AZ::ReflectContext* context);
        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;

    private:
        void InitializePid();
        void InitializeCurrentPosition();
        void InitializePosition();
        void KeepStillPosition(const uint64_t deltaTimeNs);
        void ExecuteTrajectory(const uint64_t deltaTimeNs);
        float ComputeFFJointVelocity(const float currentPosition, const float desiredPosition, const rclcpp::Duration & duration) const;
        float ComputePIDJointVelocity(const float currentPosition, const float desiredPosition, const uint64_t & deltaTimeNs, int & jointIndex);
        void SetJointVelocity(const AZ::EntityComponentIdPair idPair, const float desiredVelocity);

        AZStd::unique_ptr<FollowJointTrajectoryActionServer> m_actionServerClass;
        AZStd::string m_ROS2ControllerName;
        bool m_initialized{false};
        bool m_initializedTrajectory{false};
        Controller m_controllerType = Controller::FeedForward;
        bool m_setInitalPose{false};
        bool m_keepStillPositionInitialize{false};
        AZStd::vector<Controllers::PidConfiguration> m_pidConfigurationVector;
        AZStd::unordered_map<AZStd::string, float> m_initialPositions;

        AZStd::unordered_map<AZ::Name, float> m_jointKeepStillPosition;
        trajectory_msgs::msg::JointTrajectory m_trajectory;
        rclcpp::Time m_timeStartingExecutionTraj;

    };
} // namespace ROS2
