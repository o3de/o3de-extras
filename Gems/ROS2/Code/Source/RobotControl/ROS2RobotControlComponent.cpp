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

#include "ROS2RobotControlComponent.h"
#include "TwistControl.h"

namespace ROS2
{
    void ROS2RobotControlComponent::Init()
    {
        // TODO - instead, create/reset robot control in Activate based on selected implementation (in the component)
        m_robotControl = std::make_unique<TwistControl>();
    }

    void ROS2RobotControlComponent::Activate()
    {
        m_robotControl->Activate(GetEntity(), m_topic);
    }

    void ROS2RobotControlComponent::Deactivate()
    {
        m_robotControl->Deactivate();
    }

    void ROS2RobotControlComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<ROS2RobotControlComponent, AZ::Component>()
                ->Version(1)
                ->Field("topic", &ROS2RobotControlComponent::m_topic)
                ;

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<ROS2RobotControlComponent>("ROS2 Robot control", "[Customizable robot control component]")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game")) // TODO - "Simulation"?
                    ->DataElement(AZ::Edit::UIHandlers::Default, &ROS2RobotControlComponent::m_topic, "Topic", "ROS2 topic to subscribe to")
                    ;
            }
        }
    }

    /*
    void ROS2RobotControlComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        // TODO - query current/selected RobotControl implementation for what components are required
    }
    */
}  // namespace ROS2