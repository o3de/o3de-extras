/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include "Clock/SimulationClock.h"
#include "ROS2/ROS2Bus.h"
#include <Atom/RPI.Public/Pass/PassSystemInterface.h>
#include <AzCore/Component/Component.h>
#include <AzCore/Component/TickBus.h>
#include <AzCore/std/smart_ptr/unique_ptr.h>
#include <builtin_interfaces/msg/time.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

namespace ROS2
{
    //! Central singleton-like System Component for ROS2 Gem.
    class ROS2SystemComponent
        : public AZ::Component
        , protected ROS2RequestBus::Handler
        , public AZ::TickBus::Handler
    {
    public:
        AZ_COMPONENT(ROS2SystemComponent, "{cb28d486-afa4-4a9f-a237-ac5eb42e1c87}");

        static void Reflect(AZ::ReflectContext* context);

        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);

        ROS2SystemComponent();
        ~ROS2SystemComponent();

        //! @see ROS2Requests::GetNode()
        std::shared_ptr<rclcpp::Node> GetNode() const override;

        //! @see ROS2Requests::GetROSTimestamp()
        builtin_interfaces::msg::Time GetROSTimestamp() const override;

        // TODO - rethink ownership of this one. It needs to be a singleton-like behavior, but not necessarily here
        void BroadcastTransform(const geometry_msgs::msg::TransformStamped& t, bool isDynamic) const override;

    protected:
        ////////////////////////////////////////////////////////////////////////
        // AZ::Component interface implementation
        void Init() override;
        void Activate() override;
        void Deactivate() override;
        ////////////////////////////////////////////////////////////////////////

        ////////////////////////////////////////////////////////////////////////
        // AZTickBus interface implementation
        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;
        ////////////////////////////////////////////////////////////////////////

    private:
        std::shared_ptr<rclcpp::Node> m_ros2Node;
        AZStd::shared_ptr<rclcpp::executors::SingleThreadedExecutor> m_executor;
        AZStd::unique_ptr<tf2_ros::TransformBroadcaster> m_dynamicTFBroadcaster;
        AZStd::unique_ptr<tf2_ros::StaticTransformBroadcaster> m_staticTFBroadcaster;
        SimulationClock m_simulationClock;
        //! Used for loading the pass templates of the ROS2 gem.
        void LoadPassTemplateMappings();
        AZ::RPI::PassSystemInterface::OnReadyLoadTemplatesEvent::Handler m_loadTemplatesHandler;
    };
} // namespace ROS2
