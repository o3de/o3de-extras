/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Serialization/SerializeContext.h>
#include <ROS2/Utilities/Controllers/PidConfiguration.h>
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
        AckermannDriveModel() = default;
        AckermannDriveModel(const AckermannModelLimits& limits, const ROS2::Controllers::PidConfiguration& steeringPid);

        // DriveModel overrides
        void Activate(const VehicleConfiguration& vehicleConfig) override;

        static void Reflect(AZ::ReflectContext* context);

    protected:
        // DriveModel overrides
        void ApplyState(const VehicleInputs& inputs, AZ::u64 deltaTimeNs) override;
        const VehicleModelLimits* GetVehicleLimitPtr() const override;
        AZStd::pair<AZ::Vector3, AZ::Vector3> GetVelocityFromModel() override;

    private:
        void ApplySteering(float steering, AZ::u64 deltaTimeNs);
        void ApplySpeed(float speed, AZ::u64 deltaTimeNs);
        void ApplyWheelSteering(SteeringDynamicsData& wheelData, float steering, double deltaTimeNs);

        VehicleConfiguration m_vehicleConfiguration;
        AZStd::vector<WheelDynamicsData> m_driveWheelsData;
        AZStd::vector<SteeringDynamicsData> m_steeringData;
        ROS2::Controllers::PidConfiguration m_steeringPid;
        float m_speedCommand = 0.0f;
        AckermannModelLimits m_limits;
    };
} // namespace ROS2::VehicleDynamics
