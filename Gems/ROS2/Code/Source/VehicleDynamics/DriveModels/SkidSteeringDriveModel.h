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
        // Data collected once and cached for the reuse
        struct SkidSteeringWheelData
        {
            WheelControllerComponent* wheelControllerComponentPtr{ nullptr };
            WheelDynamicsData wheelData;
            float wheelPosition{ 0.0f }; // normalized distance between the wheel and the axle's center
            float dX{ 0.0f }; // wheel's contribution to vehicle's linear movement
            float dPhi{ 0.0f }; // wheel's contribution to vehicle's rotational movement
            AZ::Vector3 axis{ AZ::Vector3::CreateZero() }; // rotation axis of the wheel
        };
        AZStd::vector<SkidSteeringWheelData> m_wheelsCache;
        bool m_initialized = false;

        //! Collect all necessary data to compute the impact of wheels on the vehicle's velocity.
        //! The data is stored in cache and reused.
        void InitializeCache();

        //! Collect all necessary data to compute the impact of the wheel on the vehicle's velocity.
        //! It can be thought of as a column of the Jacobian matrix of the mechanical system. Jacobian matrix for this model is a matrix
        //! of size 2 x number of wheels. This function returns elements of column that corresponds to the given wheel and cache
        //! necessary data to find the wheel's rotation as a scalar.
        //! @param wheelId - id of the currently processed wheel in the axle vector
        //! @param axle - the wheel's axle configuration
        //! @param axlesCount - number of axles in the vehicle
        //! @returns A structure containing of:
        //!  - a pointer to WheelControllerComponent (API to query for wheel's ration speed),
        //!  - a structure with wheel dynamics data
        //!  - a signed distance from the wheel to the center of the axle
        //!  - a contribution to vehicle linear and angular velocity (elements of Jacobian matrix)
        //!  - an axis of wheel (to convert 3D rotation speed to given scalar)
        SkidSteeringWheelData ProduceWheelColumn(const int wheelId, const AxleConfiguration& axle, const int axlesCount) const;

        SkidSteeringModelLimits m_limits;
        AZStd::unordered_map<AZ::EntityId, VehicleDynamics::WheelDynamicsData> m_wheelsData;
        AZStd::vector<AZStd::tuple<VehicleDynamics::WheelControllerComponent*, AZ::Vector2, AZ::Vector3>> m_wheelColumns;
        VehicleConfiguration m_config;
        float m_currentLinearVelocity{ 0.0f };
        float m_currentAngularVelocity{ 0.0f };
    };
} // namespace ROS2::VehicleDynamics
