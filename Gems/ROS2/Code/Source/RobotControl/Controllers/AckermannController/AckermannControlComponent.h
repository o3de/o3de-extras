/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Component/Component.h>
#include <ROS2/RobotControl/Ackermann/AckermannBus.h>

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

        //////////////////////////////////////////////////////////////////////////
        // Component overrides
        void Activate() override;
        void Deactivate() override;
        //////////////////////////////////////////////////////////////////////////
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void Reflect(AZ::ReflectContext* context);

    private:
        //////////////////////////////////////////////////////////////////////////
        // AckermannNotificationBus::Handler overrides
        void AckermannReceived(const AckermannCommandStruct& angular) override;
        //////////////////////////////////////////////////////////////////////////
    };
} // namespace ROS2
