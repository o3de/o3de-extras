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
#include <AzFramework/Physics/SimulatedBodies/RigidBody.h>

namespace ROS2::VehicleDynamics
{
    //! A component responsible for control (steering, forward motion) of a single wheel
    class WheelControllerComponent : public AZ::Component
    {
    public:
        AZ_COMPONENT(WheelControllerComponent, "{AC594E08-DA5C-4B8D-9388-84D0840C177A}", AZ::Component);
        WheelControllerComponent() = default;
        WheelControllerComponent(const AZ::EntityId& steeringEntity, const float steeringScale);
        ~WheelControllerComponent() = default;

        //////////////////////////////////////////////////////////////////////////
        // Component overrides
        void Activate() override;
        void Deactivate() override;
        //////////////////////////////////////////////////////////////////////////

        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);

        static void Reflect(AZ::ReflectContext* context);

        AZ::Vector3 GetRotationVelocity();

        AZ::EntityId m_steeringEntity; //!< Rigid body to apply velocity to.
        float m_steeringScale{ 1.0f }; //!< The direction of torque applied to steering entity when steering is applied
    private:
        AzPhysics::RigidBody* m_rigidBodyPtr{ nullptr };
    };
} // namespace ROS2::VehicleDynamics
