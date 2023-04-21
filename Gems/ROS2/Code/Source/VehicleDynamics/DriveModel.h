/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include "VehicleConfiguration.h"
#include "VehicleInputs.h"
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/std/utils.h>
#include <VehicleDynamics/VehicleModelLimits.h>

namespace ROS2::VehicleDynamics
{
    //! Abstract class for turning vehicle inputs into behavior of wheels and steering elements
    class DriveModel
    {
    public:
        AZ_RTTI(DriveModel, "{1B57E83D-19BF-4403-8712-1AE98A12F0CD}");
        enum class DriveModelType
        {
            SimplifiedDriveModelType
        };

        static void Reflect(AZ::ReflectContext* context);
        virtual ~DriveModel() = default;

        //! Activate the model. Vehicle configuration is to remain the same until another Activate is called.
        //! @param vehicleConfig configuration containing axes and wheels information
        virtual void Activate(const VehicleConfiguration& vehicleConfig) = 0;

        //! Applies inputs to the drive. This model will calculate and apply physical forces.
        //! @param inputs captured state of inputs to use.
        //! @param deltaTimeNs nanoseconds passed since last call of this function.
        void ApplyInputState(const VehicleInputs& inputs, AZ::u64 deltaTimeNs);

        //! Computes expected velocity from individual wheels velocity.
        //! The method queries all wheels for rotation speed, and computes vehicle's expected velocity in its coordinate frame.
        //! @returns pair of linear and angular velocities
        virtual AZStd::pair<AZ::Vector3, AZ::Vector3> GetVelocityFromModel() = 0;

        //! Allows to disable vehicle dynamics.
        //! @param isDisable true if drive model should be disabled.
        void SetDisabled(bool isDisable);

        //! Get vehicle maximum limits.
        VehicleInputs GetMaximumPossibleInputs() const;

    protected:
        //! Returns pointer to implementation specific Vehicle limits.
        virtual const VehicleModelLimits* GetVehicleLimitPtr() const = 0;

        //! Apply input to implemented vehicle model.
        virtual void ApplyState(const VehicleInputs& inputs, AZ::u64 deltaTimeNs) = 0;

        //! True if model is disabled.
        bool m_disabled{ false };
    };
} // namespace ROS2::VehicleDynamics
