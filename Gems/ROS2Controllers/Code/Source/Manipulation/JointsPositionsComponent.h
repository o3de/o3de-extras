/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root
 * of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Component/Component.h>
#include <AzCore/Component/EntityBus.h>
#include <AzCore/Component/TickBus.h>
#include <AzCore/std/containers/vector.h>
#include <AzCore/std/string/string.h>

#include <ROS2/Communication/TopicConfiguration.h>
#include <ROS2/Manipulation/JointsManipulationRequests.h>
#include <ROS2/RobotControl/ControlSubscriptionHandler.h>
#include <std_msgs/msg/float64_multi_array.hpp>

namespace ROS2
{
    //! This component implements finger gripper functionality.
    class JointsPositionsComponent
        : public AZ::Component
        , public AZ::TickBus::Handler
    {
    public:
        JointsPositionsComponent() = default;
        JointsPositionsComponent(const TopicConfiguration& topicConfiguration, const AZStd::vector<AZStd::string>& jointNames);
        ~JointsPositionsComponent() = default;
        AZ_COMPONENT(JointsPositionsComponent, "{21335907-767e-4b2c-81c0-6f0b410b6d87}", AZ::Component);

        // AZ::Component overrides...
        void Activate() override;
        void Deactivate() override;

        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);

        static void Reflect(AZ::ReflectContext* context);

    private:
        // AZ::TickBus::Handler overrides...
        void OnTick(float delta, AZ::ScriptTimePoint timePoint) override;

        void ProcessPositionControlMessage(const std_msgs::msg::Float64MultiArray& message);

        AZStd::unique_ptr<IControlSubscriptionHandler> m_jointPositionsSubscriptionHandler;
        TopicConfiguration m_topicConfiguration; //!< Configuration of the subscribed topic.
        AZStd::vector<AZStd::string> m_jointNames; //!< Ordered list of joint names that can be modified via subscriber
        AZ::EntityId m_rootOfArticulation; //!< The root of the articulation chain
    };
} // namespace ROS2
