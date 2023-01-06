/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Time/ITime.h>
#include <type_traits>

namespace ROS2::VehicleDynamics
{
    //! Inputs with an expiration date - effectively is zero after a certain time since update
    template<typename T, typename = AZStd::enable_if_t<AZStd::is_arithmetic_v<T>>>
    class InputZeroedOnTimeout
    {
    public:
        InputZeroedOnTimeout(int64_t timeoutUs = 200000)
            : m_timeoutUs(timeoutUs)
        {
        }

        void UpdateValue(T updatedInput)
        {
            m_input = updatedInput;
            m_lastUpdateUs = GetTimeSinceStartupUs();
        }

        T GetValue() const
        {
            if (auto timeSinceStartUs = GetTimeSinceStartupUs(); timeSinceStartUs - m_lastUpdateUs > m_timeoutUs)
            {
                return 0;
            }
            return m_input;
        }

    private:
        int64_t GetTimeSinceStartupUs() const
        {
            return static_cast<int64_t>(AZ::Interface<AZ::ITime>::Get()->GetElapsedTimeUs());
        }

        int64_t m_timeoutUs;
        int64_t m_lastUpdateUs = 0;
        T m_input = 0;
    };

    //! Structure defining the most recent vehicle inputs state
    struct VehicleInputsState
    {
        InputZeroedOnTimeout<float> m_speed; //!< Speed measured in m/s
        InputZeroedOnTimeout<float> m_steering; //!< Steering angle in radians. Negative is right, positive is left,
    };
} // namespace ROS2::VehicleDynamics
