/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#include <AzCore/Serialization/EditContext.h>
#include <ROS2/Manipulation/MotorizedJoints/JointMotorControllerConfiguration.h>

namespace ROS2
{
    bool JointMotorControllerConfiguration::IsDebugModeVisible() const
    {
        return !m_isDebugController;
    }

    void JointMotorControllerConfiguration::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<JointMotorControllerConfiguration>()->Version(0)->Field(
                "DebugMode", &JointMotorControllerConfiguration::m_debugMode);

            if (AZ::EditContext* ec = serializeContext->GetEditContext())
            {
                ec->Class<JointMotorControllerConfiguration>("Joint Motor Controller Configuration", "Motor Controller Configuration")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default, &JointMotorControllerConfiguration::m_debugMode, "Debug Mode", "Enable Debug Mode")
                    ->Attribute(AZ::Edit::Attributes::Visibility, &JointMotorControllerConfiguration::IsDebugModeVisible);
            }
        }
    }
} // namespace ROS2
