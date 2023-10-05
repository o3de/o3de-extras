/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/RTTI/RTTI.h>
#include <AzCore/Serialization/SerializeContext.h>

namespace ROS2
{
    struct JointMotorControllerConfiguration
    {
        AZ_TYPE_INFO(JointMotorControllerConfiguration, "{4358971c-36bd-4e13-8427-74ebba6c6760}");
        static void Reflect(AZ::ReflectContext* context);

        bool IsDebugModeVisible() const;

        bool m_isDebugController{ false }; //!< Is it a debug controller.

        bool m_debugMode{ false }; //!< Is debug mode activated.
    };

} // namespace ROS2
