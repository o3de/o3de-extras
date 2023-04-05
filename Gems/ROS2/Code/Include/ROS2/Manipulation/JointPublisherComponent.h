#pragma once

#include <AzCore/Component/Component.h>
#include <AzCore/Component/TickBus.h>
#include <Source/HingeJointComponent.h>
#include <rclcpp/publisher.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace ROS2
{
    //! A component responsible for publishing the joint positions on ROS2 /joint_states topic.
    //!< @see <a href="https://docs.ros2.org/latest/api/sensor_msgs/msg/JointState.html">jointState message</a>.
    class JointPublisherComponent
        : public AZ::Component
        , public AZ::TickBus::Handler
    {
    public:
        AZ_COMPONENT(JointPublisherComponent, "{a679c2e4-a602-46de-8db4-4b33d83317f4}", AZ::Component);
        JointPublisherComponent() = default;

        //////////////////////////////////////////////////////////////////////////
        // Component overrides
        void Activate() override;
        void Deactivate() override;
        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;
        //////////////////////////////////////////////////////////////////////////
        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void Reflect(AZ::ReflectContext* context);

        AZStd::unordered_map<AZ::Name, PhysX::HingeJointComponent> &GetHierarchyMap();

    private:
        void PublishMessage();
        void UpdateMessage();
        void Initialize();
        AZStd::string GetFrameID() const;

        float GetJointPosition(const AZ::Component& hingeComponent) const;

        AZStd::unordered_map<AZ::Name, PhysX::HingeJointComponent> m_hierarchyMap;
        std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::JointState>> m_jointstatePublisher;
        sensor_msgs::msg::JointState m_jointstateMsg;
        bool m_initialized{false};
        float m_timeElapsedSinceLastTick = 0.0f;

        //! Frequency in Hz (1/s).
        float m_frequency = 10;
    };
} // namespace ROS2
