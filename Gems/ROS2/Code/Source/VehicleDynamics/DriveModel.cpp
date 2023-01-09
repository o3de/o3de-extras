/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "DriveModel.h"
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>
#include <AzCore/Serialization/SerializeContext.h>

namespace ROS2::VehicleDynamics
{
    void DriveModel::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<DriveModel>()->Version(1);
            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<DriveModel>("Drive Model", "Abstract class for drive models")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "");
            }
        }
    }


    void DriveModel::ApplyInputState(const VehicleInputsState& inputs, uint64_t deltaTimeNs){
        const VehicleModelLimits * limits = GetVehicleLimits();
        AZ_Assert(limits, "Implementation of DriveModel should return limits ");
        const VehicleInputsState filteredInputs = limits->LimitState(inputs);
        ApplyState(filteredInputs, deltaTimeNs);
    }

} // namespace ROS2::VehicleDynamics
