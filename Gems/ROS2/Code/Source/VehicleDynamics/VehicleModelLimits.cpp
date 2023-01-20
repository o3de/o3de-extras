/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "VehicleModelLimits.h"
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>
#include <AzCore/Serialization/SerializeContext.h>

namespace ROS2::VehicleDynamics
{
    void VehicleModelLimits::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<VehicleModelLimits>()->Version(2);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<VehicleModelLimits>("Vehicle Model Limits", "Limitations of speed, steering angles and other values")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2");
            }
        }
    }

    float VehicleModelLimits::LimitValue(float value, float absoluteLimit)
    {
        float absoluteLimitAbs = AZStd::abs(absoluteLimit);
        return AZStd::clamp(value, -absoluteLimitAbs, absoluteLimitAbs);
    }
} // namespace ROS2::VehicleDynamics
