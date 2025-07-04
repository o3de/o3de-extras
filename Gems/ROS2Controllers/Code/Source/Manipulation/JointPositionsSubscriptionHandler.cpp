/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "JointPositionsSubscriptionHandler.h"

namespace ROS2Controllers
{
    JointPositionsSubscriptionHandler::JointPositionsSubscriptionHandler(MessageCallback messageCallback)
        : m_messageCallback(AZStd::move(messageCallback))
    {
    }

    void JointPositionsSubscriptionHandler::SendToBus(const MessageType& message)
    {
        m_messageCallback(message);
    }
} // namespace ROS2Controllers
