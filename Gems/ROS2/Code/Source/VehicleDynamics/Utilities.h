/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include "AxleConfiguration.h"
#include "VehicleConfiguration.h"
#include "WheelDynamicsData.h"
#include <AzCore/Component/EntityId.h>
#include <AzCore/std/containers/vector.h>
#include <AzCore/std/string/string.h>

namespace ROS2::VehicleDynamics::Utilities
{
    //! Create the most common two wheel axle out of existing wheel entities.
    //! @param leftWheel Left wheel entity. It needs a WheelControllerComponent if it is a drive or steering axis.
    //! @param rightWheel Right wheel entity. It needs a WheelControllerComponent if it is a drive or steering axis.
    //! @param tag Additional information for the user (and reflected in the configuration) about what this axle is.
    //! @param wheelRadius Radius for both wheels, in meters. Wheels with different radii are not supported.
    //! @param steering Is this axle used for vehicle steering. If true, wheels need to have steering entities set in
    //! WheelControlComponents.
    //! @param drive Is this axle used to drive vehicle?
    //! @returns An axle configuration created according to call parameters.
    AxleConfiguration Create2WheelAxle(
        AZ::EntityId leftWheel, AZ::EntityId rightWheel, AZStd::string tag, float wheelRadius, bool steering, bool drive);

    //! Create an axle for both steering and drive, named "Front". @see Create2WheelAxle.
    //! @param leftWheel left wheel entity's id (entity should has rigid body and collider component)
    //! @param rightWheel right wheel entity's id (entity should has rigid body and collider component)
    //! @param wheelRadius radius in meters
    AxleConfiguration CreateFrontSteerAndDriveAxle(AZ::EntityId leftWheel, AZ::EntityId rightWheel, float wheelRadius);

    //! Create an axle for drive, named "Rear". @see Create2WheelAxle.
    //! @param leftWheel left wheel entity's id (entity should has rigid body and collider component)
    //! @param rightWheel right wheel entity's id (entity should has rigid body and collider component)
    //! @param wheelRadius radius in meters
    AxleConfiguration CreateRearDriveAxle(AZ::EntityId leftWheel, AZ::EntityId rightWheel, float wheelRadius);

    //! Retrieve all steering entities for a given vehicle configuration.
    //! @param vehicleConfig Vehicle configuration to process.
    //! @returns This function will only return data for properly set up steering entities and raise warnings if something is not right.
    //! Wheels with a WheelControllerComponent need a SteeringEntity set, and the axle must be a steering axle.
    AZStd::vector<VehicleDynamics::SteeringDynamicsData> GetAllSteeringEntitiesData(const VehicleConfiguration& vehicleConfig);

    //! Retrieve all drive entities for a given vehicle configuration.
    //! @param vehicleConfig Vehicle configuration to process.
    //! @returns This function will only return data for properly set up wheels and raise warnings if something is not right.
    //! Wheels need a WheelControllerComponent, and the axle must be a drive axle.
    AZStd::vector<VehicleDynamics::WheelDynamicsData> GetAllDriveWheelsData(const VehicleConfiguration& vehicleConfig);

    //! Get Physx Hinge Joint Component AZ::EntityComponentIdPair from Entity
    //! @param wheelEntityId id of entity that has Physx Hinge Joint Controller
    //! @returns EntityComponentIdPair that contains id of Hinge component and @param wheelEntityId
    AZ::EntityComponentIdPair GetWheelPhysxHinge(const AZ::EntityId wheelEntityId);

    //! Computes ramped velocity.
    //! @param targetVelocity Last commanded velocity to send to robot (in eg m/s or rad/s)
    //! @param lastVelocity Last commanded Velocity (in eg m/s or rad/s)
    //! @param deltaTimeNs Duration between subsequent calls to @fn ComputeRampVelocity in nanoseconds
    //! @param acceleration Acceleration (in eg m/s² or rad/s²)
    //! @param maxVelocity Limit for velocity to clamp to (in eg m/s or rad/s)
    //! @returns ramped velocity according to time, acceleration and clamped to @param maxVelocity
    float ComputeRampVelocity(float targetVelocity, float lastVelocity, AZ::u64 deltaTimeNs, float acceleration, float maxVelocity);

} // namespace ROS2::VehicleDynamics::Utilities
