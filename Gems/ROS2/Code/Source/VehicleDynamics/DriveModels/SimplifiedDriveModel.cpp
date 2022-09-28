/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "SimplifiedDriveModel.h"
#include "VehicleDynamics/Utilities.h"
#include <AzCore/Component/TransformBus.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzFramework/Physics/RigidBodyBus.h>

namespace VehicleDynamics
{
    void SimplifiedDriveModel::Reflect(AZ::ReflectContext* context)
    {
        PidConfiguration::Reflect(context);
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<SimplifiedDriveModel, DriveModel>()
                ->Version(1)
                ->Field("SteeringPID", &SimplifiedDriveModel::m_steeringPid)
                ->Field("SpeedPID", &SimplifiedDriveModel::m_speedPid);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<SimplifiedDriveModel>("Simplified Drive Model", "Configuration of a simplified vehicle dynamics drive model")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &SimplifiedDriveModel::m_steeringPid,
                        "Steering PID",
                        "Configuration of steering PID controller")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &SimplifiedDriveModel::m_speedPid,
                        "Speed PID",
                        "Configuration of speed PID controller");
            }
        }
    }

    void SimplifiedDriveModel::Activate(const VehicleConfiguration& vehicleConfig)
    {
        m_driveWheelsData.clear();
        m_steeringData.clear();
        m_vehicleConfiguration = vehicleConfig;
        m_speedPid.InitializePid();
        m_steeringPid.InitializePid();
    }

    void SimplifiedDriveModel::ApplyInputState(const VehicleInputsState& inputs, uint64_t deltaTimeNs)
    {
        if (m_driveWheelsData.empty())
        {
            m_driveWheelsData = VehicleDynamics::Utilities::GetAllDriveWheelsData(m_vehicleConfiguration);
        }

        if (m_steeringData.empty())
        {
            m_steeringData = VehicleDynamics::Utilities::GetAllSteeringEntitiesData(m_vehicleConfiguration);
        }

        ApplySteering(inputs.m_steering.GetValue(), deltaTimeNs);
        ApplySpeed(inputs.m_speed.GetValue(), deltaTimeNs);
    }

    // TODO - speed and steering handling is quite similar, possible to refactor?
    void SimplifiedDriveModel::ApplySteering(float steering, uint64_t deltaTimeNs)
    {
        if (m_steeringData.empty())
        {
            AZ_Warning("ApplySteering", false, "Cannot apply steering since no steering elements are defined in the model");
            return;
        }

        const double deltaTimeSec = double(deltaTimeNs) / 1e9;
        for (const auto& steeringElementData : m_steeringData)
        {
            auto steeringEntity = steeringElementData.m_steeringEntity;
            AZ::Vector3 currentSteeringElementRotation;
            AZ::TransformBus::EventResult(currentSteeringElementRotation, steeringEntity, &AZ::TransformBus::Events::GetLocalRotation);
            auto currentSteeringAngle = currentSteeringElementRotation.Dot(steeringElementData.m_turnAxis);
            double pidCommand = m_steeringPid.ComputeCommand(steering - currentSteeringAngle, deltaTimeNs);
            if (AZ::IsClose(pidCommand, 0.0)) // TODO - use the third argument with some reasonable value which means "close enough"
            {
                continue;
            }

            auto torque = pidCommand * deltaTimeSec;
            AZ::Transform steeringElementTransform;
            AZ::TransformBus::EventResult(steeringElementTransform, steeringEntity, &AZ::TransformBus::Events::GetWorldTM);
            auto transformedTorqueVector = steeringElementTransform.TransformVector(steeringElementData.m_turnAxis * torque);
            Physics::RigidBodyRequestBus::Event(steeringEntity, &Physics::RigidBodyRequests::ApplyAngularImpulse, transformedTorqueVector);
        }
    }

    void SimplifiedDriveModel::ApplySpeed(float speed, uint64_t deltaTimeNs)
    {
        if (m_driveWheelsData.empty())
        {
            AZ_Warning("ApplySpeed", false, "Cannot apply speed since no diving wheels are defined in the model");
            return;
        }

        const double deltaTimeSec = double(deltaTimeNs) / 1e9;
        for (const auto& wheelData : m_driveWheelsData)
        {
            auto wheelEntity = wheelData.m_wheelEntity;
            AZ::Transform wheelTransform;
            AZ::TransformBus::EventResult(wheelTransform, wheelEntity, &AZ::TransformBus::Events::GetWorldTM);

            AZ::Transform inverseWheelTransform = wheelTransform.GetInverse();
            AZ::Vector3 currentAngularVelocity;
            Physics::RigidBodyRequestBus::EventResult(currentAngularVelocity, wheelEntity, &Physics::RigidBodyRequests::GetAngularVelocity);
            currentAngularVelocity = inverseWheelTransform.TransformVector(currentAngularVelocity);
            auto currentAngularSpeedX = currentAngularVelocity.Dot(wheelData.m_driveAxis);
            float wheelRadius = wheelData.m_wheelRadius;
            if (AZ::IsClose(wheelRadius, 0.0f))
            {
                const float defaultFloatRadius = 0.35f;
                AZ_Warning("ApplySpeed", false, "Wheel radius is zero (or too close to zero), resetting to default %f", defaultFloatRadius);
                wheelRadius = defaultFloatRadius;
            }

            auto desiredAngularSpeedX = speed / wheelRadius;
            double pidCommand = m_speedPid.ComputeCommand(desiredAngularSpeedX - currentAngularSpeedX, deltaTimeNs);
            if (AZ::IsClose(pidCommand, 0.0)) // TODO - use the third argument with some reasonable value which means "close enough"
            {
                continue;
            }

            auto impulse = pidCommand * deltaTimeSec;

            auto transformedTorqueVector = wheelTransform.TransformVector(wheelData.m_driveAxis * impulse);
            Physics::RigidBodyRequestBus::Event(wheelEntity, &Physics::RigidBodyRequests::ApplyAngularImpulse, transformedTorqueVector);
        }
    }
} // namespace VehicleDynamics
