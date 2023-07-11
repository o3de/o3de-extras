/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "VehicleModelComponent.h"
#include "DriveModels/AckermannDriveModel.h"
#include "Utilities.h"
#include "VehicleConfiguration.h"
#include "VehicleModelLimits.h"
#include <AzCore/Debug/Trace.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/std/smart_ptr/make_shared.h>
#include <AzFramework/Physics/RigidBodyBus.h>
namespace ROS2::VehicleDynamics
{
    void VehicleModelComponent::Activate()
    {
        VehicleInputControlRequestBus::Handler::BusConnect(GetEntityId());

        if (m_enableManualControl)
        {
            m_manualControlEventHandler.Activate(GetEntityId());
        }
        AZ::TickBus::Handler::BusConnect();
    }

    void VehicleModelComponent::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
        m_manualControlEventHandler.Deactivate();
        VehicleInputControlRequestBus::Handler::BusDisconnect();
    }

    void VehicleModelComponent::Reflect(AZ::ReflectContext* context)
    {
        VehicleConfiguration::Reflect(context);
        VehicleModelLimits::Reflect(context);
        DriveModel::Reflect(context);

        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<VehicleModelComponent, AZ::Component>()
                ->Version(4)
                ->Field("VehicleConfiguration", &VehicleModelComponent::m_vehicleConfiguration)
                ->Field("ManualControl", &VehicleModelComponent::m_enableManualControl);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<VehicleModelComponent>("Vehicle Model", "Customizable vehicle model component")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &VehicleModelComponent::m_vehicleConfiguration,
                        "Vehicle settings",
                        "Vehicle settings including axles and common wheel parameters")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &VehicleModelComponent::m_enableManualControl,
                        "Enable Manual Control",
                        "Enable manual control of the vehicle");
            }
        }
    }

    void VehicleModelComponent::SetTargetLinearSpeed(float speedMpsX)
    {
        m_inputsState.m_speed.UpdateValue({ speedMpsX, 0, 0 });
    }

    void VehicleModelComponent::SetTargetLinearSpeedV3(const AZ::Vector3& speedMps)
    {
        m_inputsState.m_speed.UpdateValue(speedMps);
    }

    void VehicleModelComponent::SetTargetLinearSpeedFraction(float speedFractionX)
    {
        const auto& maxState = GetDriveModel()->GetMaximumPossibleInputs();
        m_inputsState.m_speed.UpdateValue(maxState.m_speed * speedFractionX);
    }

    void VehicleModelComponent::SetTargetAccelerationFraction([[maybe_unused]] float accelerationFraction)
    {
        AZ_Error("SetTargetAccelerationFraction", false, "Not implemented");
    }

    void VehicleModelComponent::SetDisableVehicleDynamics(bool isDisable)
    {
        GetDriveModel()->SetDisabled(isDisable);
    }

    void VehicleModelComponent::SetTargetSteering(float steering)
    {
        m_inputsState.m_jointRequestedPosition.UpdateValue({ steering });
    }

    void VehicleModelComponent::SetTargetSteeringFraction(float steeringFraction)
    {
        const auto& maxState = GetDriveModel()->GetMaximumPossibleInputs();
        if (!maxState.m_jointRequestedPosition.empty())
        {
            m_inputsState.m_jointRequestedPosition.UpdateValue({ maxState.m_jointRequestedPosition.front() * steeringFraction });
        }
    }

    void VehicleModelComponent::SetTargetAngularSpeed(float rateZ)
    {
        m_inputsState.m_angularRates.UpdateValue({ 0, 0, rateZ });
    };

    void VehicleModelComponent::SetTargetAngularSpeedV3(const AZ::Vector3& rate)
    {
        m_inputsState.m_angularRates.UpdateValue(rate);
    };

    void VehicleModelComponent::SetTargetAngularSpeedFraction(float rateFractionZ)
    {
        const auto& maxState = GetDriveModel()->GetMaximumPossibleInputs();
        m_inputsState.m_angularRates.UpdateValue(maxState.m_angularRates * rateFractionZ);
    };

    void VehicleModelComponent::OnTick(float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
        const uint64_t deltaTimeNs = deltaTime * 1'000'000'000;
        GetDriveModel()->ApplyInputState(m_inputsState.GetValueCheckingDeadline(), deltaTimeNs);
    }

    AZStd::pair<AZ::Vector3, AZ::Vector3> VehicleModelComponent::GetWheelsOdometry()
    {
        return GetDriveModel()->GetVelocityFromModel();
    }
} // namespace ROS2::VehicleDynamics
