/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <ROS2Controllers/RobotControl/ControlSubscriptionHandler.h>
#include <std_msgs/msg/float64_multi_array.hpp>

namespace ROS2Controllers
{
    class JointPositionsSubscriptionHandler : public ControlSubscriptionHandler<std_msgs::msg::Float64MultiArray>
    {
    public:
        using MessageType = std_msgs::msg::Float64MultiArray;
        using MessageCallback = AZStd::function<void(const MessageType&)>;

        explicit JointPositionsSubscriptionHandler(MessageCallback messageCallback);

    private:
        // ROS2::ControlSubscriptionHandler overrides
        void SendToBus(const MessageType& message) override;

        MessageCallback m_messageCallback;
    };
} // namespace ROS2Controllers
