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
#include <VehicleDynamics/VehicleInputs.h>
#include <VehicleDynamics/WheelControllerComponent.h>
#include <VehicleDynamics/WheelDynamicsData.h>

namespace ROS2::VehicleDynamics
{
    //! A simple skid steering system implementation converting speed and steering inputs into wheel impulse and steering element torque
    class SkidSteeringDriveModel : public DriveModel
    {
    public:
        AZ_RTTI(SkidSteeringDriveModel, "{04AE1BF2-621A-46C3-B025-E0875856850D}", DriveModel);
        SkidSteeringDriveModel() = default;
        SkidSteeringDriveModel(const SkidSteeringModelLimits& limits);

        // DriveModel overrides
        void Activate(const VehicleConfiguration& vehicleConfig) override;

        static void Reflect(AZ::ReflectContext* context);

    protected:
        // DriveModel overrides
        void ApplyState(const VehicleInputs& inputs, AZ::u64 deltaTimeNs) override;
        const VehicleModelLimits* GetVehicleLimitPtr() const override;
        AZStd::pair<AZ::Vector3, AZ::Vector3> GetVelocityFromModel() override;

    private:
        //! Collect all necessary data to compute the impact of the wheel on the vehicle's velocity.
        //! It can be thought of as a column of the Jacobian matrix of the mechanical system. Jacobian matrix for this model is a matrix of
        //! size 2 x number of wheels. This function returns elements of column that corresponds to the given wheel and cache necessary data
        //! to find the wheel's rotation as a scalar.
        //! @param wheelNumber - number of wheels in axis
        //! @param axle - the wheel's axle configuration
        //! @param axisCount - number of axles in the vehicle
        //! @returns A tuple containing of :
        //!  - pointer to WheelControllerComponent (API to query for wheel's ration speed),
        //!  - a contribution to vehicle linear and angular velocity (elements of Jacobian matrix)
        //!  - the axis of wheel (to convert 3D rotation speed to given scalar
        AZStd::tuple<VehicleDynamics::WheelControllerComponent*, AZ::Vector2, AZ::Vector3> ProduceWheelColumn(
            int wheelNumber, const AxleConfiguration& axle, const int axisCount) const;

        SkidSteeringModelLimits m_limits;
        AZStd::unordered_map<AZ::EntityId, VehicleDynamics::WheelDynamicsData> m_wheelsData;
        AZStd::vector<AZStd::tuple<VehicleDynamics::WheelControllerComponent*, AZ::Vector2, AZ::Vector3>> m_wheelColumns;
        VehicleConfiguration m_config;
        float m_currentLinearVelocity = 0.0f;
        float m_currentAngularVelocity = 0.0f;
    };
} // namespace ROS2::VehicleDynamics
