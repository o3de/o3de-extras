/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

namespace VehicleDynamics
{
    //! Data structure to pass wheel dynamics data for a single wheel entity.
    struct WheelDynamicsData
    {
        AZ::EntityId m_wheelEntity; //!< An entity which is expected to have a WheelControllerComponent.
        AZ::Vector3 m_driveAxis{ AZ::Vector3::CreateAxisZ() }; //!< An axis of force application for the wheel to move forward.
        float m_wheelRadius{ 0.25f }; //!< Radius of the wheel in meters.
    };

    //! Data structure to pass steering dynamics data for a single steering entity.
    struct SteeringDynamicsData
    {
        AZ::EntityId m_steeringEntity; //!< Steering entity needs to be connected (directly or indirectly) by a Joint with a wheelEntity.
        AZ::Vector3 m_turnAxis{
            AZ::Vector3::CreateAxisZ()
        }; //!< An axis of force application for the steering element to turn the attached wheel sideways.
    };
} // namespace VehicleDynamics
