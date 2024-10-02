/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/RTTI/TypeInfo.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <VehicleDynamics/VehicleModelLimits.h>

namespace ROS2::VehicleDynamics
{

    //! A structure holding limits of skid-steering robot.
    class SkidSteeringModelLimits : public VehicleModelLimits
    {
    public:
        AZ_RTTI(SkidSteeringModelLimits, "{23420EFB-BB62-48C7-AD37-E50580A53C39}", VehicleModelLimits);
        SkidSteeringModelLimits() = default;

        static void Reflect(AZ::ReflectContext* context);

        // VehicleModelLimits overrides
        VehicleInputs LimitState(const VehicleInputs& inputState) const;
        VehicleInputs GetMaximumState() const;

        float GetLinearAcceleration() const;
        float GetAngularAcceleration() const;
        float GetLinearSpeedLimit() const;
        float GetAngularSpeedLimit() const;

        void SetAngularAccelerationLimit(const float limit);

    private:
        float m_linearLimit = 2.0f; //!< [m/s] Maximum travel velocity.
        float m_angularLimit = 3.5f; //!< [Rad/s] Maximum rotation speed.
        float m_linearAcceleration = 3.5f; //!< [m*s^(-2)] Linear acceleration limit
        float m_angularAcceleration = 2.0f; //!< [rad*s^(-2)] Angular acceleration limit
    };
} // namespace ROS2::VehicleDynamics
