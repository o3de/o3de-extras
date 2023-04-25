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
#include <AzCore/std/utils.h>

namespace ROS2::VehicleDynamics
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
        //! @param speedMps is a linear speed vectors, unit is meters per second.
        virtual void SetTargetLinearSpeedV3(const AZ::Vector3& speedMps) = 0;

        //! Set target for the vehicle linear speed. It should be realized over time according to drive model.
        //! @param speedMpsX is a linear speed in meters per second with the plus sign in the forward direction.
        virtual void SetTargetLinearSpeed(float speedMpsX) = 0;

        //! Steer in a direction given in relative coordinate system (current direction is 0).
        //! @param steering is angle in radians, positive to the right and negative to the left.
        //! @note The actual angle applied is subject to limits and implementation (eg smoothing).
        virtual void SetTargetSteering(float steering) = 0;

        //! Set target for the angular speed. It should be realized over time according to drive model.
        //! @param rate is an angular speed vector, unit is radians per second.
        virtual void SetTargetAngularSpeedV3(const AZ::Vector3& rate) = 0;

        //! Set target for the angular speed. It should be realized over time according to drive model.
        //! @param rateZ is an angular speed in radians per second in up direction .
        virtual void SetTargetAngularSpeed(float rateZ) = 0;

        //! Accelerate without target speed, relative to the limits.
        //! @param accelerationFraction is relative to limits of possible acceleration.
        //! 1 - accelerate as much as possible, -1 - brake as much as possible.
        virtual void SetTargetAccelerationFraction(float accelerationFraction) = 0;

        //! Steer input version which is relative to limits.
        //! @param steeringFraction is -1 to 1, which applies as a fraction of vehicle model steering limits.
        //! @note The actual angle applied is subject to limits and implementation (eg smoothing).
        virtual void SetTargetSteeringFraction(float steeringFraction) = 0;

        //! Speed input version which is relative to limits.
        //! @param speedFractionX is -1 to 1, which applies as a fraction of vehicle model speed limits.
        virtual void SetTargetLinearSpeedFraction(float speedFractionX) = 0;

        //! Set target for the angular speed. It should be realized over time according to drive model.
        //! @param rateFractionZ is an angular speed in radians per second in up direction, fraction of maximum speed.
        virtual void SetTargetAngularSpeedFraction(float rateFractionZ) = 0;

        //! Disables (or enables) the vehicle dynamics
        //! @param disable if set true no torque will be applied
        virtual void SetDisableVehicleDynamics(bool disable) = 0;

        //! Returns vehicle's velocity in its local coordinate system
        //! @returns pair of Vectors : Linear speed (in m/s) and angular speed (in rad/s)
        virtual AZStd::pair<AZ::Vector3, AZ::Vector3> GetWheelsOdometry() = 0;
    };

    using VehicleInputControlRequestBus = AZ::EBus<VehicleInputControlRequests>;
} // namespace ROS2::VehicleDynamics