#pragma once

#include <AzCore/Component/Component.h>
#include <AzFramework/Components/TransformComponent.h>

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "LidarTemplate.h"
#include "LidarRaycaster.h"

namespace ROS2
{
    class ROS2LidarSensorComponent
        : public AZ::Component
        , public AZ::TickBus::Handler
    {
    public:
        AZ_COMPONENT(ROS2LidarSensorComponent, "{502A955F-7742-4E23-AD77-5E4063739DCA}", AZ::Component);

        // AZ::Component interface implementation.
        void Init() override;
        void Activate() override;
        void Deactivate() override;

        // Required Reflect function.
        static void Reflect(AZ::ReflectContext* context);

        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;
        // Optional functions for defining provided and dependent services.
        // static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        // static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        // static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);

    private:
        AZStd::string m_frameName = "test_lidar";
        LidarTemplate::LidarModel m_lidarModel = LidarTemplate::SickMRS6000;
        float m_hz = 10;
        float m_frameTime;
        LidarRaycaster m_lidarRaycaster;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_pointCloudPublisher;
        AzFramework::TransformComponent* entityTransform;
    };
}  // namespace ROS2