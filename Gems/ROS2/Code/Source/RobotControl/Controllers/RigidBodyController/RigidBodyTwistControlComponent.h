/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include "ROS2/RobotControl/Twist/TwistBus.h"
#include <AzCore/Component/Component.h>

namespace ROS2
{
    //! A component with a simple handler for Twist type of control (linear and angular velocities).
    //! Velocities are directly applied to a selected body.
    class RigidBodyTwistControlComponent
        : public AZ::Component
        , private TwistNotificationBus::Handler
    {
    public:
        AZ_COMPONENT(RigidBodyTwistControlComponent, "{D994FE1A-AA6A-42B9-8B8E-B3B375891F5B}", AZ::Component);
        RigidBodyTwistControlComponent() = default;

        void Activate() override;
        void Deactivate() override;
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void Reflect(AZ::ReflectContext* context);

    private:
        //! Simplest approach: To imitate the steering, current linear and angular velocities of a rigid body are overwritten with inputs
        void TwistReceived(const AZ::Vector3& linear, const AZ::Vector3& angular) override;
    };
} // namespace ROS2
