/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Component/Component.h>
#include <AzCore/Component/EntityId.h>
#include <AzCore/Math/Vector3.h>

namespace VehicleDynamics
{
    //! A component responsible for control (steering, forward motion) of a single wheel
    class WheelControllerComponent : public AZ::Component
    {
    public:
        AZ_COMPONENT(WheelControllerComponent, "{AC594E08-DA5C-4B8D-9388-84D0840C177A}", AZ::Component);
        WheelControllerComponent() = default;
        ~WheelControllerComponent() = default;

        void Activate() override;
        void Deactivate() override;
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);

        static void Reflect(AZ::ReflectContext* context);

        AZ::EntityId m_steeringEntity; //!< Rigid body to apply torque to. TODO - parent, this entity or custom.
        AZ::Vector3 m_driveDir{ 0.0, 0.0, 1.0 }; //!< The direction of torque applied to wheel entity when speed is applied
        AZ::Vector3 m_steeringDir{ 0.0, 0.0, 1.0 }; //!< The direction of torque applied to steering entity when steering is applied
    };
} // namespace VehicleDynamics
