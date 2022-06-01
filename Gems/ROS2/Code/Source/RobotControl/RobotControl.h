/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include "Frame/ROS2FrameComponent.h"
#include "RobotControl/ControlConfiguration.h"
#include "RobotControl/RobotConfiguration.h"
#include "ROS2/ROS2Bus.h"
#include "Utilities/ROS2Names.h"
#include <memory>
#include <rclcpp/rclcpp.hpp>

namespace ROS2
{
    //! Component extension enabling polymorphic use of generics.
    // TODO - naming (this is a component activation interface capturing component and entity state)
    class IRobotControl
    {
    public:
        //! Activate the Control with the Component activation.
        //! Only activated IRobotControl will receive and process control messages.
        //! @param entity Activation context for the owning Component - the entity it belongs to.
        virtual void Activate(const AZ::Entity* entity) = 0;
        virtual void Deactivate() = 0;
        virtual ~IRobotControl() = default;
    };

    //! The generic class for handling subscriptions to ROS2 control messages of different types.
    //! @see ControlConfiguration::Steering.
    template <typename T>
    class RobotControl : public IRobotControl
    {
    public:
        explicit RobotControl(ControlConfiguration controlConfiguration)
            : m_controlConfiguration{std::move(controlConfiguration)} {}

        void Activate(const AZ::Entity* entity) final
        {
            m_active = true;
            if (!m_controlSubscription)
            {
                auto ros2Frame = entity->FindComponent<ROS2FrameComponent>();
                AZStd::string namespacedTopic = ROS2Names::GetNamespacedName(
                        ros2Frame->GetNamespace(),
                        m_controlConfiguration.m_topic);

                auto ros2Node = ROS2Interface::Get()->GetNode();
                m_controlSubscription = ros2Node->create_subscription<T>(
                    namespacedTopic.data(),
                    m_controlConfiguration.m_qos.GetQoS(),
                    [this](const T& message) { OnControlMessage(message); });
            }
        };

        void Deactivate() final
        {
            m_active = false;
            m_controlSubscription.reset(); // Note: topic and qos can change, need to re-subscribe
        };

        virtual ~RobotControl() = default;

    protected:
        ControlConfiguration m_controlConfiguration;

    private:
        void OnControlMessage(const T& message)
        {
            if (!m_active) return;

            if (m_controlConfiguration.m_broadcastBusMode)
            {
                BroadcastBus(message);
            } 
            else
            {
                ApplyControl(message);
            }
        };

        virtual void ApplyControl(const T& message) = 0;
        virtual void BroadcastBus(const T& message) = 0;

        bool m_active = false;

        typename rclcpp::Subscription<T>::SharedPtr m_controlSubscription;
    };
}  // namespace ROS2
