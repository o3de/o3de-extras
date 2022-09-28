/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include "VehicleConfiguration.h"
#include "VehicleInputsState.h"
#include <AzCore/Serialization/SerializeContext.h>

namespace VehicleDynamics
{
    //! Abstract class for turning vehicle inputs into behavior of wheels and steering elements
    class DriveModel
    {
    public:
        AZ_RTTI(DriveModel, "{1B57E83D-19BF-4403-8712-1AE98A12F0CD}");
        enum DriveModelType
        {
            SimplifiedDriveModelType
        };

        static void Reflect(AZ::ReflectContext* context);
        virtual ~DriveModel() = default;
        virtual DriveModel::DriveModelType DriveType() = 0;

        //! Activate the model. Vehicle configuration is to remain the same until another Activate is called.
        virtual void Activate(const VehicleConfiguration& vehicleConfig) = 0;

        //! Applies inputs to the drive. This model will calculate and apply physical forces.
        //! @param inputs captured state of inputs to use
        //! @param deltaTimeNs nanoseconds passed since last call of this function.
        virtual void ApplyInputState(const VehicleInputsState& inputs, uint64_t deltaTimeNs) = 0;
    };
} // namespace VehicleDynamics
