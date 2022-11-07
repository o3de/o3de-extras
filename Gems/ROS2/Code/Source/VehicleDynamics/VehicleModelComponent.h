/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include "ManualControlEventHandler.h"
#include "ROS2/VehicleDynamics/VehicleInputControlBus.h"
#include "VehicleConfiguration.h"
#include "VehicleDynamics/DriveModels/AckermannDriveModel.h"
#include "VehicleInputsState.h"
#include "VehicleModelLimits.h"
#include <AzCore/Component/Component.h>
#include <AzCore/Component/TickBus.h>
#include <AzCore/std/smart_ptr/unique_ptr.h>

namespace VehicleDynamics
{
    //! A central vehicle (and robot) dynamics component, which can be extended with additional modules.
    class VehicleModelComponent
        : public AZ::Component
        , private VehicleInputControlRequestBus::Handler
        , private AZ::TickBus::Handler
    {
    public:
        AZ_COMPONENT(VehicleModelComponent, "{7093AE7A-9F64-4C77-8189-02C6B7802C1A}", AZ::Component);
        VehicleModelComponent() = default;

        void Activate() override;
        void Deactivate() override;
        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void Reflect(AZ::ReflectContext* context);

    private:
        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;

        //! @see VehicleInputControlRequests
        void SetTargetLinearSpeed(float speedMps) override;
        void SetTargetSteering(float steering) override;
        void SetTargetAccelerationFraction(float accelerationFraction) override;
        void SetTargetSteeringFraction(float steeringFraction) override;
        void SetTargetLinearSpeedFraction(float speedFraction) override;
        void SetDisableVehicleDynamics(bool is_disable) override;

        ManualControlEventHandler m_manualControlEventHandler;
        VehicleConfiguration m_vehicleConfiguration;
        VehicleInputsState m_inputsState;
        AckermannDriveModel m_driveModel; // TODO - use abstraction here (DriveModel)
        VehicleModelLimits m_vehicleLimits;
        // TODO - Engine, Transmission, Lights, etc.
    };
} // namespace VehicleDynamics
