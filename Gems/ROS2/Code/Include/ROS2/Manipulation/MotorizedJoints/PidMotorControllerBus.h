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

namespace ROS2
{
    //! Interface to communicate with a PID motor controller.
    //! It allows to apply a setpoint and track performance of the controller.
    class PidMotorControllerRequests : public AZ::EBusTraits
    {
    public:
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::ById;
        using BusIdType = AZ::EntityId;

        virtual ~PidMotorControllerRequests() = default;

        //! Set current setpoint value for position controller.
        //! The setpoint is the desired position for a simulated actuator.
        //! For linear actuators, the setpoint is given in meters.
        //! For the rotational actuators, the setpoint is given in radians.
        //! @param setpoint value in meters or radians
        virtual void SetSetpoint(float setpoint) = 0;

        //! Get current setpoint value for position controller.
        //! If `SetSetpoint` has not been called, it returns initial position of joint.
        //! For linear actuators, the setpoint is given in meters.
        //! For the rotational actuators, the setpoint is given in radians.
        //! @returns setpoint value in meters or radians
        virtual float GetSetpoint() = 0;

        //! Retrieve current measurement.
        //! Measurement is the value of the movement - eg protrusion of an actuator.
        //! For linear actuators, measurement is given in meters.
        //! For the rotational actuators, measurement is given in radians.
        //! When the setpoint is reached this should be close to setpoint.
        //! @returns measurement value in meters or radians.
        virtual float GetCurrentMeasurement() = 0;

        //! Retrieve current control error (difference between setpoint and measurement)
        //! When the setpoint is reached this should be close to zero.
        //! For linear actuators, the error is given in meters.
        //! For the rotational actuators, the error is given in radians.
        //! @returns controller error's value in meters or radians
        virtual float GetError() = 0;
    };

    using PidMotorControllerRequestBus = AZ::EBus<PidMotorControllerRequests>;
} // namespace ROS2
