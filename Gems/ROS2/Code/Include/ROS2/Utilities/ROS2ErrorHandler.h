/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Debug/Trace.h>
#include <exception>
#include <functional>

namespace ROS2
{
    //! Utility class for handling ROS2 exceptions. Tracks the state of a component.
    class ROS2ErrorHandler
    {
    public:
        //! Returns false if component state is ROS2ComponentState::Error, true otherwise.
        bool IsComponentValid();

        //! Sets m_componentState for ROS2ComponentState::Error
        void SetComponentStateError();

        //! Executes function @param ros2Operation and catches possible exceptions caused by it.
        //! When an exception occurs, it prints an error message on the console
        //! containing @param componentName, @param contextMessage and error message.
        void ExecuteROS2Context(std::function<void()> ros2Operation, const char* componentName, const char* contextMessage);

    private:
        // Describes component's state based on whether a ROS 2 error has occurred.
        enum class ROS2ComponentState
        {
            Ok, ///< Correctly initialized and working component.
            Error ///< An error occurred in component.
        };
        ROS2ComponentState m_componentState{ ROS2ComponentState::Ok };
    };
} // namespace ROS2
