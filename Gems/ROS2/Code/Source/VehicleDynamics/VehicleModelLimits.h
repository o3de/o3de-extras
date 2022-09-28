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

namespace VehicleDynamics
{
    //! A structure holding limits of vehicle, including speed and steering limits
    struct VehicleModelLimits
    {
    public:
        AZ_TYPE_INFO(VehicleModelLimits, "{76DA392D-64BB-45A8-BC90-84AAE7901991}");
        VehicleModelLimits() = default;
        static void Reflect(AZ::ReflectContext* context);

        //! Limit value with a symmetrical range.
        //! @param value Input value.
        //! @param absoluteLimit Limits for value (between -absoluteLimit and absoluteLimit).
        //! @returns A limited value. Always returns either value, -absoluteLimit or absoluteLimit.
        static float LimitValue(float value, float absoluteLimit);

        float m_speedLimit = 10.0f; //!< [Mps] Applies to absolute value
        float m_steeringLimit = 0.7f; //!< [rad] Applies to absolute value
    };
} // namespace VehicleDynamics
