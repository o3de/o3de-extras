/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/Component/Component.h>
#include <ROS2/RobotControl/Twist/TwistBus.h>
#include <VehicleDynamics/AxleConfiguration.h>
#include <VehicleDynamics/Utilities.h>

namespace ROS2
{

    //! Component that contains skid steering model.
    class SkidSteeringControlComponent
        : public AZ::Component
        , private TwistNotificationBus::Handler
    {
    public:
        AZ_COMPONENT(SkidSteeringControlComponent, "{7FEE7851-1284-4AE5-9C2C-763916BFE641}", AZ::Component);
        SkidSteeringControlComponent() = default;

        // Component overrides
        void Activate() override;
        void Deactivate() override;

        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void Reflect(AZ::ReflectContext* context);

    private:
        // TwistNotificationBus::Handler overrides
        void TwistReceived(const AZ::Vector3& linear, const AZ::Vector3& angular) override;
    };
} // namespace ROS2
