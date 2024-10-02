/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "SkidSteeringModelLimits.h"
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>
#include <AzCore/Serialization/SerializeContext.h>

namespace ROS2::VehicleDynamics
{
    void SkidSteeringModelLimits::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<SkidSteeringModelLimits>()
                ->Version(1)
                ->Field("LinearLimit", &SkidSteeringModelLimits::m_linearLimit)
                ->Field("AngularLimit", &SkidSteeringModelLimits::m_angularLimit)
                ->Field("LinearAcceleration", &SkidSteeringModelLimits::m_linearAcceleration)
                ->Field("AngularAcceleration", &SkidSteeringModelLimits::m_angularAcceleration);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<SkidSteeringModelLimits>("Skid steering Model Limits", "Limitations of speed, steering angles and other values")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &SkidSteeringModelLimits::m_linearLimit,
                        "Linear speed Limit",
                        "Max linear speed (meters/sec)")
                    ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                    ->Attribute(AZ::Edit::Attributes::Max, 100.0f)
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &SkidSteeringModelLimits::m_angularLimit,
                        "Angular speed Limit",
                        "Max angular speed (rad/s)")
                    ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                    ->Attribute(AZ::Edit::Attributes::Max, 10.0f)
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &SkidSteeringModelLimits::m_angularAcceleration,
                        "Angular acceleration",
                        "Acceleration in rad/s²")
                    ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                    ->Attribute(AZ::Edit::Attributes::Max, 100.0f)
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &SkidSteeringModelLimits::m_linearAcceleration,
                        "Linear acceleration",
                        "Acceleration in m/s²")
                    ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                    ->Attribute(AZ::Edit::Attributes::Max, 100.0f);
            }
        }
    }

    VehicleInputs SkidSteeringModelLimits::LimitState(const VehicleInputs& inputState) const
    {
        VehicleInputs ret = inputState;
        ret.m_speed = AZ::Vector3{ LimitValue(ret.m_speed.GetX(), m_linearLimit), 0.f, 0.f };
        ret.m_angularRates = AZ::Vector3{ 0.f, 0.f, LimitValue(ret.m_angularRates.GetZ(), m_angularLimit) };
        return ret;
    }

    VehicleInputs SkidSteeringModelLimits::GetMaximumState() const
    {
        VehicleInputs ret;
        ret.m_speed = { m_linearLimit, 0, 0 };
        ret.m_angularRates = { 0, 0, m_angularLimit };
        return ret;
    }

    float SkidSteeringModelLimits::GetLinearAcceleration() const
    {
        return m_linearAcceleration;
    }

    float SkidSteeringModelLimits::GetAngularAcceleration() const
    {
        return m_angularAcceleration;
    }

    float SkidSteeringModelLimits::GetLinearSpeedLimit() const
    {
        return m_linearLimit;
    }

    float SkidSteeringModelLimits::GetAngularSpeedLimit() const
    {
        return m_angularLimit;
    }

    void SkidSteeringModelLimits::SetAngularAccelerationLimit(const float limit)
    {
        m_angularAcceleration = AZStd::clamp(limit, 0.0f, 100.0f);
    }

} // namespace ROS2::VehicleDynamics
