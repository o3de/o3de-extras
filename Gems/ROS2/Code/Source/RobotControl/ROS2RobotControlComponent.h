/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include "ControlConfiguration.h"
#include "ControlSubscriptionHandler.h"
#include <AzCore/Component/Component.h>
#include <AzCore/std/smart_ptr/unique_ptr.h>
#include <AzToolsFramework/ToolsComponents/GenericComponentWrapper.h>

namespace ROS2
{
    //! A Component responsible for controlling a robot movement.
    //! Uses IRobotControl implementation depending on type of ROS2 control message.
    //! Depends on ROS2FrameComponent. Can be configured through ControlConfiguration.
    class ROS2RobotControlComponent : public AZ::Component
    {
    public:
        AZ_COMPONENT(ROS2RobotControlComponent, "{CBFB0764-99F9-40EE-9FEE-F5F5A66E59D2}", AZ::Component);
        ROS2RobotControlComponent() = default;
        ROS2RobotControlComponent(ControlConfiguration controlConfiguration)
            : m_controlConfiguration(AZStd::move(controlConfiguration))
        {
        }

        const ControlConfiguration& GetControlConfiguration() const;

        void SetControlConfiguration(const ControlConfiguration& controlConfiguration);

        // AZ::Component interface implementation.
        void Activate() override;
        void Deactivate() override;
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);

        // Required Reflect function.
        static void Reflect(AZ::ReflectContext* context);

    private:
        AZStd::unique_ptr<IControlSubscriptionHandler> m_subscriptionHandler;
        ControlConfiguration m_controlConfiguration;
    };
} // namespace ROS2
