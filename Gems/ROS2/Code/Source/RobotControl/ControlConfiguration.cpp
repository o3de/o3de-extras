/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <AzCore/Serialization/EditContext.h>
#include <ROS2/RobotControl/ControlConfiguration.h>

namespace ROS2
{
    void ControlConfiguration::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<ControlConfiguration>()->Version(1)->Field("Steering", &ControlConfiguration::m_steering);

            if (AZ::EditContext* ec = serializeContext->GetEditContext())
            {
                ec->Class<ControlConfiguration>("Robot control", "Handles robot control")
                    ->DataElement(
                        AZ::Edit::UIHandlers::ComboBox,
                        &ControlConfiguration::m_steering,
                        "Steering",
                        "Determines how the robot is controlled.")
                    ->Attribute(AZ::Edit::Attributes::ChangeNotify, AZ::Edit::PropertyRefreshLevels::EntireTree)
                    ->EnumAttribute(ControlConfiguration::Steering::Twist, "Twist")
                    ->EnumAttribute(ControlConfiguration::Steering::Ackermann, "Ackermann");
            }
        }
    }
} // namespace ROS2
