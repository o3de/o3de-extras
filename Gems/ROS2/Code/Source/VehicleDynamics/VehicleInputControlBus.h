/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Component/EntityId.h>
#include <AzCore/EBus/EBus.h>
#include <AzCore/Interface/Interface.h>

namespace VehicleDynamics
{
    //! Inputs (speed, steering, acceleration etc.) for vehicle dynamics system
    //! Inputs are valid for a short time (configurable) and need to be repeated if continuous movement is needed.
    //! (Use cruise system to set cruise speed).
    class VehicleInputControlRequests : public AZ::EBusTraits
    {
    public:
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::ById;
        using BusIdType = AZ::EntityId;

        virtual ~VehicleInputControlRequests() = default;

        //! Set target for the vehicle linear speed. It should be realized over time according to drive model.
        //! @param speedMps is a linear speed in meters per second with the plus sign in the forward direction.
        virtual void SetTargetLinearSpeed(float speedMps) = 0;

        //! Steer in a direction given in relative coordinate system (current direction is 0).
        //! @param steering is angle in radians, positive to the right and negative to the left.
        //! Note that the actual angle applied is subject to limits and implementation (e.g. smoothing).
        virtual void SetTargetSteering(float steering) = 0;

        //! Accelerate without target speed, relative to the limits.
        //! @param acceleration is relative to limits of possible acceleration.
        //! 1 - accelerate as much as possible, -1 - brake as much as possible.
        virtual void SetTargetAccelerationFraction(float accelerationFraction) = 0;

        //! Steer input version which is relative to limits.
        //! @param steering is -1 to 1, which applies as a fraction of vehicle model steering limits.
        //! Note that the actual angle applied is subject to limits and implementation (e.g. smoothing).
        virtual void SetTargetSteeringFraction(float steeringFraction) = 0;

        //! Speed input version which is relative to limits.
        //! @param speedMps is -1 to 1, which applies as a fraction of vehicle model speed limits.
        virtual void SetTargetLinearSpeedFraction(float speedFraction) = 0;
    };

    using VehicleInputControlRequestBus = AZ::EBus<VehicleInputControlRequests>;
} // namespace VehicleDynamics