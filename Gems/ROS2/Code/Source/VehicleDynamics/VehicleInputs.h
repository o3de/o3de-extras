/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Math/Vector3.h>
#include <AzCore/Time/ITime.h>
#include <AzCore/std/containers/vector.h>

namespace ROS2::VehicleDynamics
{
    //! Inputs with an expiration date - effectively is zero after a certain time since update

    template<typename T>
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

        T& GetValue()
        {
            if (auto timeSinceStartUs = GetTimeSinceStartupUs(); timeSinceStartUs - m_lastUpdateUs > m_timeoutUs)
            {
                m_input = Zero(m_input);
            }
            return m_input;
        }

    private:
        T& Zero(T& input);
        int64_t GetTimeSinceStartupUs() const
        {
            return static_cast<int64_t>(AZ::Interface<AZ::ITime>::Get()->GetElapsedTimeUs());
        }

        int64_t m_timeoutUs;
        int64_t m_lastUpdateUs = 0;
        T m_input{ 0 };
    };

    //! Structure defining the most recent vehicle inputs state
    struct VehicleInputs
    {
        AZ::Vector3 m_speed; //!< Linear speed control measured in m/s
        AZ::Vector3 m_angularRates; //!< Angular speed control of vehicle
        AZStd::vector<float> m_jointRequestedPosition; //!< Steering angle in radians. Negative is right, positive is left,
    };

    struct VehicleInputDeadline
    {
        InputZeroedOnTimeout<AZ::Vector3> m_speed; //!< Linear speed control measured in m/s
        InputZeroedOnTimeout<AZ::Vector3> m_angularRates; //!< Linear speed control measured in m/s
        InputZeroedOnTimeout<AZStd::vector<float>>
            m_jointRequestedPosition; //!< Steering angle in radians. Negative is right, positive is left,
        VehicleInputs GetValueCheckingDeadline();
    };

} // namespace ROS2::VehicleDynamics
