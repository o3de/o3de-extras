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
    //! A structure holding limits of vehicle, including speed and steering limits
    class AckermannModelLimits : public VehicleModelLimits
    {
    public:
        AZ_RTTI(AckermannModelLimits, "{796928D9-436F-472D-B841-E67F28F9CC82}", VehicleModelLimits);
        AckermannModelLimits() = default;
        AckermannModelLimits(const float speedLimit, const float steeringLimit, const float acceleration);
        static void Reflect(AZ::ReflectContext* context);

        // VehicleModelLimits overrides
        VehicleInputs LimitState(const VehicleInputs& inputState) const;
        VehicleInputs GetMaximumState() const;

        float GetLinearAcceleration() const;
        float GetLinearSpeedLimit() const;

    private:
        float m_speedLimit = 10.0f; //!< [Mps] Applies to absolute value
        float m_steeringLimit = 0.7f; //!< [rad] Applies to absolute value
        float m_acceleration = 10.0f; //!< [m*s^(-2)] Linear acceleration limit
    };
} // namespace ROS2::VehicleDynamics
