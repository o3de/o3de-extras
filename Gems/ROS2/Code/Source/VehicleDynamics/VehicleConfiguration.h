/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include "AxleConfiguration.h"
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/std/containers/vector.h>
#include <AzFramework/Physics/Material/PhysicsMaterial.h>

namespace ROS2::VehicleDynamics
{
    //! Drive and steering configuration for a vehicle. The class only holds axle information now but it is meant to be expanded.
    class VehicleConfiguration
    {
    public:
        AZ_TYPE_INFO(VehicleConfiguration, "{C616E333-E618-4E37-8CE6-1E8A28182D00}");
        VehicleConfiguration() = default;
        static void Reflect(AZ::ReflectContext* context);

        AZStd::vector<AxleConfiguration> m_axles; //!> Axles of the vehicle, front to rear
        float m_wheelbase = 2.0; //!> The distance in meters between the midpoints of vehicles front and rear wheels
        float m_track = 1.0; //!> The distance in meters between the hub flanges on an axle
    };
} // namespace ROS2::VehicleDynamics
