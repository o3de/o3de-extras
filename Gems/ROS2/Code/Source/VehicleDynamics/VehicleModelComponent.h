/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include "DriveModels/AckermannDriveModel.h"
#include "ManualControlEventHandler.h"
#include "VehicleConfiguration.h"
#include "VehicleInputsState.h"
#include <VehicleDynamics/VehicleModelLimits.h>
#include <AzCore/Component/Component.h>
#include <AzCore/Component/TickBus.h>
#include <AzCore/std/smart_ptr/unique_ptr.h>
#include <ROS2/VehicleDynamics/VehicleInputControlBus.h>

namespace ROS2::VehicleDynamics
{
    //! A central vehicle (and robot) dynamics component, which can be extended with additional modules.
    class VehicleModelComponent
        : public AZ::Component
        , private VehicleInputControlRequestBus::Handler
        , private AZ::TickBus::Handler
    {
    public:
        AZ_RTTI(VehicleModelComponent, "{7093AE7A-9F64-4C77-8189-02C6B7802C1A}", AZ::Component);
        VehicleModelComponent() = default;

        //////////////////////////////////////////////////////////////////////////
        // Component overrides
        void Activate() override;
        void Deactivate() override;
        //////////////////////////////////////////////////////////////////////////

        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void Reflect(AZ::ReflectContext* context);

    private:
        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;

        //////////////////////////////////////////////////////////////////////////
        // VehicleInputControlRequestBus::Handler overrides
        void SetTargetLinearSpeedX(float speedMps) override;
        void SetTargetLinearSpeed(AZ::Vector3 speedMps) override;
        void SetTargetSteering(float steering) override;
        void SetTargetAngularSpeedZ(float rate) override;
        void SetTargetAngularSpeed(AZ::Vector3 rate) override;
        void SetTargetAccelerationFraction(float accelerationFraction) override;
        void SetTargetSteeringFraction(float steeringFraction) override;
        void SetTargetLinearSpeedXFraction(float speedFraction) override;
        void SetTargetAngularSpeedZFraction(float rateFraction) override;
        void SetDisableVehicleDynamics(bool isDisable) override;
        //////////////////////////////////////////////////////////////////////////


    protected:
        ManualControlEventHandler m_manualControlEventHandler;
        VehicleInputsStateTimeouted m_inputsState;
        VehicleDynamics::VehicleConfiguration m_vehicleConfiguration;
        virtual DriveModel * GetDriveModel() =0;


    };
} // namespace ROS2::VehicleDynamics
