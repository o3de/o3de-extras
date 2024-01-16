/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "AckermannModelLimits.h"
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>
#include <AzCore/Serialization/SerializeContext.h>

namespace ROS2::VehicleDynamics
{
    void AckermannModelLimits::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<AckermannModelLimits>()
                ->Version(1)
                ->Field("SpeedLimit", &AckermannModelLimits::m_speedLimit)
                ->Field("SteeringLimit", &AckermannModelLimits::m_steeringLimit)
                ->Field("Acceleration", &AckermannModelLimits::m_acceleration);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<AckermannModelLimits>("Skid steering Model Limits", "Limitations of speed, steering angles and other values")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default, &AckermannModelLimits::m_speedLimit, "Speed Limit", "Max linear speed (meters/sec)")
                    ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                    ->Attribute(AZ::Edit::Attributes::Max, 100.0f)
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default, &AckermannModelLimits::m_steeringLimit, "Steering Limit", "Max steering angle (rad)")
                    ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                    ->Attribute(AZ::Edit::Attributes::Max, 1.57f)
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default, &AckermannModelLimits::m_acceleration, "Acceleration", "Acceleration in m/s²")
                    ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                    ->Attribute(AZ::Edit::Attributes::Max, 100.0f);
            }
        }
    }

    AckermannModelLimits::AckermannModelLimits(const float speedLimit, const float steeringLimit, const float acceleration)
    {
        m_speedLimit = AZStd::clamp(speedLimit, 0.0f, 100.0f);
        m_steeringLimit = AZStd::clamp(steeringLimit, 0.0f, 1.57f);
        m_acceleration = AZStd::clamp(acceleration, 0.0f, 100.0f);
    }

    VehicleInputs AckermannModelLimits::LimitState(const VehicleInputs& inputState) const
    {
        VehicleInputs ret = inputState;
        ret.m_angularRates = AZ::Vector3{ 0.0f };
        ret.m_speed = AZ::Vector3{ LimitValue(ret.m_speed.GetX(), m_speedLimit), 0.0f, 0.0f };
        if (!ret.m_jointRequestedPosition.empty())
        {
            ret.m_jointRequestedPosition.front() = LimitValue(ret.m_jointRequestedPosition.front(), m_steeringLimit);
        }
        return ret;
    }

    VehicleInputs AckermannModelLimits::GetMaximumState() const
    {
        VehicleInputs ret;
        ret.m_speed = { m_speedLimit, 0.0f, 0.0f };
        ret.m_jointRequestedPosition = { m_steeringLimit };
        return ret;
    }

    float AckermannModelLimits::GetLinearSpeedLimit() const
    {
        return m_speedLimit;
    }

    float AckermannModelLimits::GetLinearAcceleration() const
    {
        return m_acceleration;
    }

} // namespace ROS2::VehicleDynamics
