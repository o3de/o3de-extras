/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <ROS2/ROS2Bus.h>

namespace ROS2
{
    // Component extension enabling polymorphic use of generics
    // TODO - naming (this is a component activation interface capturing component and entity state)
    class IRobotControl
    {
    public:
        virtual void Activate(const AZ::Entity* entity, const AZStd::string& topicName, const QoS &qos) = 0;
        virtual void Deactivate() = 0;
        virtual ~IRobotControl() = default;
    };

    template <typename T>
    class RobotControl : public IRobotControl
    {
    public:
        // TODO - pass component args (qos, topic name, etc) as editor-enabled serializable struct
        void Activate(const AZ::Entity* entity, const AZStd::string& topicName, const QoS &qos) final
        {
            SetTargetComponent(entity);
            m_active = true;
            if (!m_controlSubscription)
            {
                auto ros2Node = ROS2Interface::Get()->GetNode();
                m_controlSubscription = ros2Node->create_subscription<T>(topicName.data(), qos.GetQoS(),
                    std::bind(&RobotControl<T>::OnControlMessage, this, std::placeholders::_1));
            }
        };

        void Deactivate() final
        {
            m_active = false;
            m_controlSubscription.reset(); // Note: topic and qos can change, need to re-subscribe
        };

    private:
        void OnControlMessage(const T& message)
        {
            if (m_active)
            {
                ApplyControl(message);
            }
        };

        virtual void ApplyControl(const T& message) = 0;

        // Subclass implementations will extract necessary services or components to manipulate
        virtual void SetTargetComponent(const AZ::Entity* entity) = 0;

        bool m_active = false;
        std::string m_topicName;

        typename rclcpp::Subscription<T>::SharedPtr m_controlSubscription;
    };
}  // namespace ROS2