/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "VehicleInputs.h"

namespace ROS2::VehicleDynamics
{
    template<>
    AZ::Vector3& InputZeroedOnTimeout<AZ::Vector3>::Zero(AZ::Vector3& input)
    {
        input = AZ::Vector3::CreateZero();
        return input;
    }

    template<>
    AZStd::vector<float>& InputZeroedOnTimeout<AZStd::vector<float>>::Zero(AZStd::vector<float>& input)
    {
        AZStd::fill(input.begin(), input.end(), 0);
        return input;
    }

    VehicleInputs VehicleInputDeadline::GetValueCheckingDeadline()
    {
        return VehicleInputs{ m_speed.GetValue(), m_angularRates.GetValue(), m_jointRequestedPosition.GetValue() };
    }
} // namespace ROS2::VehicleDynamics
