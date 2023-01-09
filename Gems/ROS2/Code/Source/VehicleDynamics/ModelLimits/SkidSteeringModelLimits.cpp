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
                ->Field("AngularLimit", &SkidSteeringModelLimits::m_angularLimit);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<SkidSteeringModelLimits>("Skid steering Model Limits", "Limitations of speed, steering angles and other values")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &SkidSteeringModelLimits::m_linearLimit,
                        "Linear speed Limit",
                        "Max linear speed (mps)")
                    ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                    ->Attribute(AZ::Edit::Attributes::Max, 100.0f)
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &SkidSteeringModelLimits::m_angularLimit,
                        "Angular speed Limit",
                        "Max angular sped (rad/s)")
                    ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                    ->Attribute(AZ::Edit::Attributes::Max, 10.0f);
            }
        }
    }

    VehicleInputsState SkidSteeringModelLimits::LimitState(const VehicleInputsState& inputState) const
    {
        VehicleInputsState ret = inputState;
        const auto& v = ret.m_speed;
        ret.m_speed = AZ::Vector3{ LimitValue(v.GetX(), m_linearLimit), 0.f, 0.f };
        const auto& r = ret.m_angularRates;
        ret.m_angularRates = AZ::Vector3{ 0.f, 0.f, LimitValue(r.GetZ(), m_angularLimit) };
        return ret;
    }

    VehicleInputsState SkidSteeringModelLimits::GetMaximumState() const
    {
        VehicleInputsState ret;
        ret.m_speed = { m_linearLimit, 0, 0 };
        ret.m_angularRates = { 0, 0, m_angularLimit };
        return ret;
    }

} // namespace ROS2::VehicleDynamics
