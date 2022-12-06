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
#include <AzCore/Debug/Trace.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzFramework/Physics/RigidBodyBus.h>

namespace VehicleDynamics
{
    void VehicleModelComponent::Activate()
    {
        VehicleInputControlRequestBus::Handler::BusConnect(GetEntityId());
        m_manualControlEventHandler.Activate(GetEntityId());
        AZ::TickBus::Handler::BusConnect();
        m_driveModel.Activate(m_vehicleConfiguration);
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
        DriveModel::Reflect(context);
        AckermannDriveModel::Reflect(context);
        VehicleModelLimits::Reflect(context);
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<VehicleModelComponent, AZ::Component>()
                ->Version(3)
                ->Field("VehicleConfiguration", &VehicleModelComponent::m_vehicleConfiguration)
                ->Field("DriveModel", &VehicleModelComponent::m_driveModel)
                ->Field("VehicleModelLimits", &VehicleModelComponent::m_vehicleLimits);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<VehicleModelComponent>("Vehicle Model", "Customizable vehicle model component")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &VehicleModelComponent::m_vehicleConfiguration,
                        "Vehicle settings",
                        "Vehicle settings including axles and common wheel parameters")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &VehicleModelComponent::m_driveModel,
                        "Drive model",
                        "Settings of the selected drive model")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &VehicleModelComponent::m_vehicleLimits,
                        "Vehicle limits",
                        "Limits for parameters such as speed and steering angle");
            }
        }
    }

    void VehicleModelComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("VehicleModelService"));
    }

    void VehicleModelComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    { // Only one per entity
        incompatible.push_back(AZ_CRC_CE("VehicleModelService"));
    }

    void VehicleModelComponent::SetTargetLinearSpeed(float speedMps)
    {
        auto limitedSpeed = VehicleModelLimits::LimitValue(speedMps, m_vehicleLimits.m_speedLimit);
        m_inputsState.m_speed.UpdateValue(limitedSpeed);
    }

    void VehicleModelComponent::SetTargetLinearSpeedFraction(float speedFraction)
    {
        m_inputsState.m_speed.UpdateValue(speedFraction * m_vehicleLimits.m_speedLimit);
    }

    void VehicleModelComponent::SetTargetAccelerationFraction([[maybe_unused]] float accelerationFraction)
    {
        AZ_Error("SetTargetAccelerationFraction", false, "Not implemented");
    }

    void VehicleModelComponent::SetDisableVehicleDynamics(bool isDisable)
    {
        m_driveModel.SetDisabled(isDisable);
    }

    void VehicleModelComponent::SetTargetSteering(float steering)
    {
        auto limitedSteering = VehicleModelLimits::LimitValue(steering, m_vehicleLimits.m_steeringLimit);
        m_inputsState.m_steering.UpdateValue(limitedSteering);
    }

    void VehicleModelComponent::SetTargetSteeringFraction(float steeringFraction)
    {
        m_inputsState.m_steering.UpdateValue(steeringFraction * m_vehicleLimits.m_steeringLimit);
    }

    void VehicleModelComponent::OnTick(float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
        const uint64_t deltaTimeNs = deltaTime * 1'000'000'000;
        m_driveModel.ApplyInputState(m_inputsState, deltaTimeNs);
    }
} // namespace VehicleDynamics
