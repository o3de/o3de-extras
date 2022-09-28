/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "RobotControl/ControlConfiguration.h"
#include "Utilities/ROS2Names.h"
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>

namespace ROS2
{
    void ControlConfiguration::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<ControlConfiguration>()
                ->Version(1)
                ->Field("Topic", &ControlConfiguration::m_topic)
                ->Field("Qos", &ControlConfiguration::m_qos)
                ->Field("Steering", &ControlConfiguration::m_steering);

            if (AZ::EditContext* ec = serializeContext->GetEditContext())
            {
                ec->Class<ControlConfiguration>("Robot control", "Handles robot control")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &ControlConfiguration::m_topic, "Topic", "ROS2 topic to subscribe to")
                    ->Attribute(AZ::Edit::Attributes::ChangeValidate, &ROS2Names::ValidateTopicField)
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default, &ControlConfiguration::m_qos, "QoS", "Quality of Service settings for subscriber")
                    ->DataElement(
                        AZ::Edit::UIHandlers::ComboBox,
                        &ControlConfiguration::m_steering,
                        "Steering",
                        "Determines how robot is controlled.")
                    ->Attribute(AZ::Edit::Attributes::ChangeNotify, AZ::Edit::PropertyRefreshLevels::EntireTree)
                    ->EnumAttribute(ControlConfiguration::Steering::Twist, "Twist")
                    ->EnumAttribute(ControlConfiguration::Steering::Ackermann, "Ackermann");
            }
        }
    }
} // namespace ROS2
