/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include "RobotControl/Ackermann/AckermannBus.h"
#include <AzCore/Component/Component.h>

namespace ROS2
{
    //! A simple component which translates ackermann commands to vehicle dynamics inputs
    class AckermannControlComponent
        : public AZ::Component
        , private AckermannNotificationBus::Handler
    {
    public:
        AZ_COMPONENT(AckermannControlComponent, "{16EC2F18-F579-414C-8B3B-DB47078729BC}", AZ::Component);
        AckermannControlComponent() = default;

        void Activate() override;
        void Deactivate() override;
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void Reflect(AZ::ReflectContext* context);

    private:
        //! Simply relay received commands to vehicle dynamics input system
        void AckermannReceived(const AckermannCommandStruct& angular) override;
    };
} // namespace ROS2
