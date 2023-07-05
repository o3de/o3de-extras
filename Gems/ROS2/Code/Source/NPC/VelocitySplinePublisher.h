/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Component/Component.h>
#include <AzCore/Component/Entity.h>
#include <AzCore/Component/TickBus.h>

#include <AzCore/Math/Spline.h>
#include <AzCore/Math/Transform.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/std/containers/deque.h>

#include <AzCore/Component/EntityBus.h>
#include <AzFramework/Entity/EntityDebugDisplayBus.h>
#include <geometry_msgs/msg/twist.hpp>

namespace ROS2
{
    //! Component that send velocity commands to ROS2 system according to entity position and spline trajectory.
    class VelocitySplinePublisher
        : public AZ::Component
        , private AZ::TickBus::Handler
        , private AZ::EntityBus::Handler
        , private AzFramework::EntityDebugDisplayEventBus::Handler
    {
    public:
        AZ_COMPONENT(VelocitySplinePublisher, "{28b8b025-b499-4876-bc06-1b9112ff62d3}");
        VelocitySplinePublisher() = default;
        ~VelocitySplinePublisher() override = default;

        static void Reflect(AZ::ReflectContext* context);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);

        // AZ::Component overrides ...
        void Activate() override;
        void Deactivate() override;

        // AZ::TickBus::Handler overrides ...
        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;

        // AZ::EntityBus::Handler overrides ...
        void OnEntityActivated(const AZ::EntityId& entityId) override;


        // AzFramework::EntityDebugDisplayEventBus::Handler overrides
        void DisplayEntityViewport(const AzFramework::ViewportInfo& viewportInfo, AzFramework::DebugDisplayRequests& debugDisplay) override;

    private:
        float GetAngle(const AZ::Vector3 &v1, const AZ::Vector3 &v2);
        float m_lookAheadDistance = 1.f;
        float m_linearSpeedFactor = 0.5f;
        float m_angularSpeedFactor = 5.f;
        float m_crossTrackFactor = 0.3f;
        bool m_drawInGame = true;

        TopicConfiguration m_cmdTopicConfiguration;
        AZ::EntityId m_baselinkEntityId;
        std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::Twist>> m_cmdPublisher;
        AZ::Transform m_idealGoal{ AZ::Transform::CreateIdentity() };
    };
} // namespace ROS2
