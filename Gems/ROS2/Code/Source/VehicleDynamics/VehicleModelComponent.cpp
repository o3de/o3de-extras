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
#include <AzFramework/Physics/RigidBodyBus.h>
#include <AzCore/std/smart_ptr/make_shared.h>
namespace ROS2::VehicleDynamics
{
    void VehicleModelComponent::Activate()
    {
        VehicleInputControlRequestBus::Handler::BusConnect(GetEntityId());
        m_manualControlEventHandler.Activate(GetEntityId());
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
                ->Field("VehicleConfiguration", &VehicleModelComponent::m_vehicleConfiguration);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<VehicleModelComponent>("Vehicle Model", "Customizable vehicle model component")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &VehicleModelComponent::m_vehicleConfiguration,
                        "Vehicle settings",
                        "Vehicle settings including axles and common wheel parameters");
            }
        }
    }

    void VehicleModelComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("VehicleModelService"));
    }

    void VehicleModelComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("VehicleModelService"));
    }

    void VehicleModelComponent::SetTargetLinearSpeedX(float speedMps)
    {
        m_inputsState.m_speed.UpdateValue({speedMps,0,0});
    }

    void VehicleModelComponent::SetTargetLinearSpeed(AZ::Vector3 speedMps)
    {
        m_inputsState.m_speed.UpdateValue(speedMps);
    }

    void VehicleModelComponent::SetTargetLinearSpeedXFraction(float speedFraction)
    {
        const auto& maxState = GetDriveModel()->GetVehicleLimits()->GetMaximumState();
        m_inputsState.m_speed.UpdateValue(maxState.m_speed * speedFraction);
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
        m_inputsState.m_jointConfiguration.UpdateValue({steering});
    }

    void VehicleModelComponent::SetTargetSteeringFraction(float steeringFraction)
    {
        const auto& maxState = GetDriveModel()->GetVehicleLimits()->GetMaximumState();
        if(!maxState.m_jointConfiguration.empty()){
            m_inputsState.m_jointConfiguration.UpdateValue({maxState.m_jointConfiguration.front() * steeringFraction});
        }
    }

    void VehicleModelComponent::SetTargetAngularSpeedZ(float rate){
        m_inputsState.m_angularRates.UpdateValue({0,0,rate});
    };

    void VehicleModelComponent::SetTargetAngularSpeed(AZ::Vector3 rate)
    {
        m_inputsState.m_angularRates.UpdateValue(rate);
    };

    void VehicleModelComponent::SetTargetAngularSpeedZFraction(float rateFraction){
        const auto& maxState = GetDriveModel()->GetVehicleLimits()->GetMaximumState();
        m_inputsState.m_angularRates.UpdateValue(maxState.m_angularRates * rateFraction);
    };

    void VehicleModelComponent::OnTick(float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
        const uint64_t deltaTimeNs = deltaTime * 1'000'000'000;
        GetDriveModel()->ApplyInputState(m_inputsState.GetTimeoutedValue(), deltaTimeNs);
    }
} // namespace ROS2::VehicleDynamics
