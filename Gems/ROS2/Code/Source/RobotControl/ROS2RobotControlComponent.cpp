/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "RobotControl/ROS2RobotControlComponent.h"
#include "RobotControl/Ackermann/AckermannSubscriptionHandler.h"
#include "RobotControl/Twist/TwistSubscriptionHandler.h"
#include <AzCore/Component/Entity.h>
#include <AzCore/Debug/Trace.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>
#include <AzCore/Serialization/SerializeContext.h>

namespace ROS2
{
    void ROS2RobotControlComponent::Activate()
    {
        switch (m_controlConfiguration.m_steering)
        {
        case ControlConfiguration::Twist:
            m_subscriptionHandler = AZStd::make_unique<TwistSubscriptionHandler>();
            break;
        case ControlConfiguration::Ackermann:
            // TODO add ackermann
            m_subscriptionHandler = AZStd::make_unique<AckermannSubscriptionHandler>();
            break;
        default:
            AZ_Error("ROS2RobotControlComponent", false, "Control type %d not implemented", m_controlConfiguration.m_steering);
            break;
        }

        if (m_subscriptionHandler)
        {
            m_subscriptionHandler->Activate(GetEntity(), m_controlConfiguration);
        }
    }

    void ROS2RobotControlComponent::Deactivate()
    {
        if (m_subscriptionHandler)
        {
            m_subscriptionHandler->Deactivate();
            m_subscriptionHandler.reset();
        }
    }

    void ROS2RobotControlComponent::Reflect(AZ::ReflectContext* context)
    {
        ControlConfiguration::Reflect(context);

        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<ROS2RobotControlComponent, AZ::Component>()->Version(1)->Field(
                "ControlConfiguration", &ROS2RobotControlComponent::m_controlConfiguration);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<ROS2RobotControlComponent>("ROS2 Robot control", "Customizable robot control component")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game")) // TODO - "Simulation"?
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ROS2RobotControlComponent::m_controlConfiguration,
                        "Control settings",
                        "Control subscription setting and type of control");
            }
        }
    }

    void ROS2RobotControlComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        // TODO - also, dependent on current/selected RobotControl implementation for what components are required
        required.push_back(AZ_CRC("ROS2Frame"));
    }

    void ROS2RobotControlComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("ROS2RobotControl"));
    }
} // namespace ROS2
