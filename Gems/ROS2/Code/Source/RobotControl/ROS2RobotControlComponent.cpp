/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Component/Entity.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>

#include "RobotControl/ROS2RobotControlComponent.h"
#include "RobotControl/TwistControl.h"
#include "Utilities/ROS2Names.h"

namespace ROS2
{
    void ROS2RobotControlComponent::Activate()
    {
        auto ros2Frame = GetEntity()->FindComponent<ROS2FrameComponent>();
        auto namespacedTopic = ROS2Names::GetNamespacedName(ros2Frame->GetNamespace(), m_topic);

        // TODO - instead, create/reset robot control in Activate based on selected implementation (in the component)
        m_robotControl = std::make_unique<TwistControl>();
        m_robotControl->Activate(GetEntity(), namespacedTopic, m_qos);
    }

    void ROS2RobotControlComponent::Deactivate()
    {
        m_robotControl->Deactivate();
        m_robotControl.reset();
    }

    void ROS2RobotControlComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<ROS2RobotControlComponent, AZ::Component>()
                ->Version(1)
                ->Field("topic", &ROS2RobotControlComponent::m_topic)
                ->Field("qos", &ROS2RobotControlComponent::m_qos)
                ;

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<ROS2RobotControlComponent>("ROS2 Robot control", "[Customizable robot control component]")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game")) // TODO - "Simulation"?
                    ->DataElement(AZ::Edit::UIHandlers::Default, &ROS2RobotControlComponent::m_topic, "Topic", "ROS2 topic to subscribe to")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &ROS2RobotControlComponent::m_qos, "QoS", "Quality of Service settings for subscriber")
                    ;
            }
        }
    }

    void ROS2RobotControlComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        // TODO - also, dependent on current/selected RobotControl implementation for what components are required
        required.push_back(AZ_CRC("ROS2Frame"));
    }
}  // namespace ROS2