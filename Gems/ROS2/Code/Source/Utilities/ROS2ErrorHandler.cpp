/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "AzCore/std/string/string.h"
#include <ROS2/Utilities/ROS2ErrorHandler.h>

namespace ROS2
{
    bool ROS2ErrorHandler::IsComponentValid()
    {
        return m_componentState == ROS2ComponentState::Ok;
    }

    void ROS2ErrorHandler::SetComponentStateError()
    {
        m_componentState = ROS2ComponentState::Error;
    }

    void ROS2ErrorHandler::ExecuteROS2Context(std::function<void()> ros2Operation, const char* componentName, const char* contextMessage)
    {
        try
        {
            ros2Operation();
        }
        catch (std::exception& e)
        {
            AZStd::string errorInfo = AZStd::string(contextMessage) + " " + e.what();
            AZ_Error(componentName, false, errorInfo.c_str());
            SetComponentStateError();
        }
    }
} // namespace ROS2
