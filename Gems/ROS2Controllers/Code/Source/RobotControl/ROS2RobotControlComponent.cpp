/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ROS2RobotControlComponent.h"
#include <AzCore/Component/Entity.h>
#include <AzCore/Debug/Trace.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>
#include <AzCore/Serialization/SerializeContext.h>
#include <RobotControl/Ackermann/AckermannSubscriptionHandler.h>
#include <RobotControl/ROS2RobotControlComponent.h>
#include <RobotControl/Twist/TwistSubscriptionHandler.h>

namespace ROS2
{
    void ROS2RobotControlComponent::Activate()
    {
        switch (m_controlConfiguration.m_steering)
        {
        case ControlConfiguration::Steering::Twist:
            m_subscriptionHandler = AZStd::make_unique<TwistSubscriptionHandler>();
            break;
        case ControlConfiguration::Steering::Ackermann:
            m_subscriptionHandler = AZStd::make_unique<AckermannSubscriptionHandler>();
            break;
        default:
            AZ_Error("ROS2RobotControlComponent", false, "Control type %d not implemented", m_controlConfiguration.m_steering);
            break;
        }

        if (m_subscriptionHandler)
        {
            m_subscriptionHandler->Activate(GetEntity(), m_subscriberConfiguration);
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
            serialize->Class<ROS2RobotControlComponent, AZ::Component>()
                ->Version(2)
                ->Field("ControlConfiguration", &ROS2RobotControlComponent::m_controlConfiguration)
                ->Field("SubscriberConfiguration", &ROS2RobotControlComponent::m_subscriberConfiguration);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<ROS2RobotControlComponent>("ROS2 Robot Control", "Customizable robot control component")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->Attribute(AZ::Edit::Attributes::Icon, "Editor/Icons/Components/ROS2RobotControl.svg")
                    ->Attribute(AZ::Edit::Attributes::ViewportIcon, "Editor/Icons/Components/Viewport/ROS2RobotControl.svg")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ROS2RobotControlComponent::m_controlConfiguration,
                        "Control settings",
                        "Control subscription setting and type of control")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ROS2RobotControlComponent::m_subscriberConfiguration,
                        "Topic subscriber configuration",
                        "Configuration of ROS2 topic Robot Control subscribes to");
            }
        }
    }

    void ROS2RobotControlComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("ROS2Frame"));
    }

    const ControlConfiguration& ROS2RobotControlComponent::GetControlConfiguration() const
    {
        return m_controlConfiguration;
    }

    const TopicConfiguration& ROS2RobotControlComponent::GetSubscriberConfiguration() const
    {
        return m_subscriberConfiguration;
    }

    void ROS2RobotControlComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("ROS2RobotControl"));
    }

    void ROS2RobotControlComponent::SetControlConfiguration(const ControlConfiguration& controlConfiguration)
    {
        m_controlConfiguration = controlConfiguration;
    }

    void ROS2RobotControlComponent::SetSubscriberConfiguration(const TopicConfiguration& subscriberConfiguration)
    {
        m_subscriberConfiguration = subscriberConfiguration;
    }

} // namespace ROS2
