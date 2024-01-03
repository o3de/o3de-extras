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
    //! A simple skid steering system implementation converting speed and steering inputs into wheel rotation
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
        //! Data to compute the impact of a single wheel on the vehicle's velocity and to access the wheel to get/set target values.
        struct SkidSteeringWheelData
        {
            WheelControllerComponent* wheelControllerComponentPtr{ nullptr }; //!< Pointer to wheel controller to set/get rotation speed.
            WheelDynamicsData wheelData; //!< Wheel parameters.
            float wheelPosition{ 0.0f }; //!< Normalized distance between the wheel and the axle's center.
            float dX{ 0.0f }; //!< Wheel's contribution to vehicle's linear movement.
            float dPhi{ 0.0f }; //!< Wheel's contribution to vehicle's rotational movement.
            AZ::Vector3 axis{ AZ::Vector3::CreateZero() }; //!< Rotation axis of the wheel.
        };
        AZStd::vector<SkidSteeringWheelData> m_wheelsData; //!< Buffer with pre-calculated wheels' data.
        bool m_initialized = false; //!< Information if m_wheelsData was pre-calculated.

        //! Compute all necessary data for wheels' contribution to the vehicle's velocity and store it in the buffer.
        void ComputeWheelsData();

        //! Compute all necessary data for a single wheel's contribution to the vehicle's velocity. It can be thought of as a column of the
        //! Jacobian matrix of the mechanical system. This function returns elements of column that corresponds to the given wheel, stores
        //! necessary data to find the wheel's rotation as a scalar, and stores necessary data to set this rotation.
        //! @param wheelId - id of the currently processed wheel in the axle vector
        //! @param axle - the wheel's axle configuration
        //! @param axlesCount - number of axles in the vehicle
        //! @returns SkidSteeringWheelData structure for a single wheel
        SkidSteeringWheelData ComputeSingleWheelData(const int wheelId, const AxleConfiguration& axle, const int axlesCount) const;

        SkidSteeringModelLimits m_limits;
        VehicleConfiguration m_config;
        float m_currentLinearVelocity{ 0.0f };
        float m_currentAngularVelocity{ 0.0f };
    };
} // namespace ROS2::VehicleDynamics
