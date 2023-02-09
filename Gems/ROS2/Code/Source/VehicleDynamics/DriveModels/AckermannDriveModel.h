/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Serialization/SerializeContext.h>
#include <ROS2/VehicleDynamics/DriveModels/PidConfiguration.h>
#include <VehicleDynamics/DriveModel.h>
#include <VehicleDynamics/ModelLimits/AckermannModelLimits.h>
#include <VehicleDynamics/VehicleConfiguration.h>
#include <VehicleDynamics/VehicleInputs.h>
#include <VehicleDynamics/WheelDynamicsData.h>

namespace ROS2::VehicleDynamics
{
    //! A simple Ackermann system implementation converting speed and steering inputs into wheel impulse and steering element torque
    class AckermannDriveModel : public DriveModel
    {
    public:
        AZ_RTTI(AckermannDriveModel, "{104AC31D-E30B-4454-BF42-4FB37B8CFD9B}", DriveModel);

        // DriveModel overrides
        void Activate(const VehicleConfiguration& vehicleConfig) override;

        static void Reflect(AZ::ReflectContext* context);

    protected:
        // DriveModel overrides
        void ApplyState(const VehicleInputs& inputs, AZ::u64 deltaTimeNs) override;
        const VehicleModelLimits* GetVehicleLimitPtr() const override;

    private:
        void ApplySteering(float steering, AZ::u64 deltaTimeNs);
        void ApplySpeed(float speed, AZ::u64 deltaTimeNs);
        void ApplyWheelSteering(SteeringDynamicsData& wheelData, float steering, double deltaTimeNs);

        VehicleConfiguration m_vehicleConfiguration;
        AZStd::vector<WheelDynamicsData> m_driveWheelsData;
        AZStd::vector<SteeringDynamicsData> m_steeringData;
        PidConfiguration m_steeringPid;
        AckermannModelLimits m_limits;
    };
} // namespace ROS2::VehicleDynamics
