/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Component/Component.h>
#include <AzCore/Component/TickBus.h>
#include <AzCore/std/smart_ptr/unique_ptr.h>
#include <Lidar/LidarSystem.h>
#include <ROS2/Clock/SimulationClock.h>
#include <ROS2/ROS2Bus.h>
#include <builtin_interfaces/msg/time.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

/**
 * \mainpage
 *
 * Welcome to the **Open 3D Engine (O3DE)** API Reference for the **ROS 2 Gem**!
 *
 * The Components and Classes of this Gem support direct interaction with the ROS 2 ecosystem.
 * The Robot Operating System (ROS) middleware documentation links in this API Reference point to the most recent ROS distribution. If you
 * are using an older version, you might need to navigate to the corresponding documentation pages for the distribution which you have
 * installed.
 *
 * For the overview, features and usage of this Gem, please refer to [ROS 2 Gem
 * Documentation](https://development--o3deorg.netlify.app/docs/user-guide/gems/reference/robotics/ros2/).
 *
 * o3de-doxygen-insert-table
 *
 */

namespace ROS2
{
    //! Central singleton-like System Component for ROS2 Gem.
    class ROS2SystemComponent
        : public AZ::Component
        , public AZ::TickBus::Handler
        , protected ROS2RequestBus::Handler
    {
    public:
        AZ_COMPONENT(ROS2SystemComponent, "{cb28d486-afa4-4a9f-a237-ac5eb42e1c87}");

        static void Reflect(AZ::ReflectContext* context);

        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);

        ROS2SystemComponent();
        ~ROS2SystemComponent() override;

        //////////////////////////////////////////////////////////////////////////
        // ROS2RequestBus::Handler overrides
        std::shared_ptr<rclcpp::Node> GetNode() const override;
        void ConnectOnNodeChanged(NodeChangedEvent::Handler& handler) override;
        builtin_interfaces::msg::Time GetROSTimestamp() const override;
        void BroadcastTransform(const geometry_msgs::msg::TransformStamped& t, bool isDynamic) override;
        const SimulationClock& GetSimulationClock() const override;
        //////////////////////////////////////////////////////////////////////////

    protected:
        ////////////////////////////////////////////////////////////////////////
        // AZ::Component override
        void Init() override;
        void Activate() override;
        void Deactivate() override;
        ////////////////////////////////////////////////////////////////////////

        ////////////////////////////////////////////////////////////////////////
        // AZTickBus interface implementation
        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;
        ////////////////////////////////////////////////////////////////////////
    private:
        void InitClock();

        std::vector<geometry_msgs::msg::TransformStamped> m_frameTransforms;

        std::shared_ptr<rclcpp::Node> m_ros2Node;
        AZStd::shared_ptr<rclcpp::executors::SingleThreadedExecutor> m_executor;
        AZStd::unique_ptr<tf2_ros::TransformBroadcaster> m_dynamicTFBroadcaster;
        AZStd::unique_ptr<tf2_ros::StaticTransformBroadcaster> m_staticTFBroadcaster;
        AZStd::unique_ptr<SimulationClock> m_simulationClock;
        NodeChangedEvent m_nodeChangedEvent;
    };
} // namespace ROS2
