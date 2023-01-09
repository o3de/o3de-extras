/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/Serialization/SerializeContext.h>
#include <VehicleDynamics/DriveModel.h>

#include <VehicleDynamics/ModelLimits/SkidSteeringModelLimits.h>
#include <VehicleDynamics/VehicleConfiguration.h>
#include <VehicleDynamics/VehicleInputsState.h>
#include <VehicleDynamics/WheelDynamicsData.h>

namespace ROS2::VehicleDynamics
{
    //! A simple Ackermann system implementation converting speed and steering inputs into wheel impulse and steering element torque
    class SkidSteeringDriveModel : public DriveModel
    {
    public:
        AZ_RTTI(SkidSteeringDriveModel, "{04AE1BF2-621A-46C3-B025-E0875856850D}", DriveModel);
        SkidSteeringDriveModel::DriveModelType DriveType() const override
        {
            return SkidSteeringDriveModel::DriveModelType::SimplifiedDriveModelType;
        }
        //////////////////////////////////////////////////////////////////////////
        // DriveModel overrides
        void Activate(const VehicleConfiguration& vehicleConfig) override;
        void SetDisabled(bool isDisabled) override;
        VehicleModelLimits* GetVehicleLimits() override;
        //////////////////////////////////////////////////////////////////////////

        static void Reflect(AZ::ReflectContext* context);

    protected:
        //////////////////////////////////////////////////////////////////////////
        // DriveModel overrides
        void ApplyState(const VehicleInputsState& inputs, uint64_t deltaTimeNs) override;
        //////////////////////////////////////////////////////////////////////////
    private:
        bool m_disabled{ false };
        SkidSteeringModelLimits m_limits;
        AZStd::unordered_map<AZ::EntityId, AZ::EntityComponentIdPair> m_wheelsData;
        VehicleConfiguration m_config;
    };
} // namespace ROS2::VehicleDynamics
