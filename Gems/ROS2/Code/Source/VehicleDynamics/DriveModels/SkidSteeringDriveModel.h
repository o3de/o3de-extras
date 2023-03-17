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
#include <VehicleDynamics/WheelDynamicsData.h>
#include <VehicleDynamics/WheelControllerComponent.h>

namespace ROS2::VehicleDynamics
{
    //! A simple Ackermann system implementation converting speed and steering inputs into wheel impulse and steering element torque
    class SkidSteeringDriveModel : public DriveModel
    {
    public:
        AZ_RTTI(SkidSteeringDriveModel, "{04AE1BF2-621A-46C3-B025-E0875856850D}", DriveModel);

        // DriveModel overrides
        void Activate(const VehicleConfiguration& vehicleConfig) override;

        static void Reflect(AZ::ReflectContext* context);

    protected:
        // DriveModel overrides
        void ApplyState(const VehicleInputs& inputs, AZ::u64 deltaTimeNs) override;
        const VehicleModelLimits* GetVehicleLimitPtr() const override;
        AZStd::pair<AZ::Vector3, AZ::Vector3> GetVelocityFromModel() override;

    private:
        SkidSteeringModelLimits m_limits;
        AZStd::unordered_map<AZ::EntityId, AZ::EntityComponentIdPair> m_wheelsData;
        AZStd::vector<AZStd::tuple<VehicleDynamics::WheelControllerComponent*, AZ::Vector2, AZ::Vector3>> m_wheelColumns;
        VehicleConfiguration m_config;
        float m_currentLinearVelocity = 0.0f;
        float m_currentAngularVelocity = 0.0f;
    };
} // namespace ROS2::VehicleDynamics
