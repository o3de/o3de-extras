/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include "VehicleInputs.h"
#include <AzCore/RTTI/TypeInfo.h>
#include <AzCore/Serialization/SerializeContext.h>

namespace ROS2::VehicleDynamics
{
    //! A structure holding limits of vehicle, including speed and steering limits
    class VehicleModelLimits
    {
    public:
        AZ_RTTI(VehicleModelLimits, "{76DA392D-64BB-45A8-BC90-84AAE7901991}");
        VehicleModelLimits() = default;
        virtual ~VehicleModelLimits() = default;
        static void Reflect(AZ::ReflectContext* context);

        //! Limit input state to values that are possible for model.
        //! @param inputState Input state to filter.
        //! @returns Filtered, pruned state.
        virtual VehicleInputs LimitState(const VehicleInputs& inputState) const = 0;

        //! Returns maximal permissible input states.
        //! @returns VehicleInputsState
        virtual VehicleInputs GetMaximumState() const = 0;

    protected:
        //! Limit value with a symmetrical range.
        //! @param value Input value.
        //! @param absoluteLimit Limits for value (between -absoluteLimit and absoluteLimit).
        //! @returns A limited value. Always returns either value, -absoluteLimit or absoluteLimit.
        static float LimitValue(float value, float absoluteLimit);
    };
} // namespace ROS2::VehicleDynamics
