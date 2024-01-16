/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "AckermannDriveModel.h"
#include <AzCore/Component/TransformBus.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzFramework/Physics/RigidBodyBus.h>
#include <HingeJointComponent.h>
#include <PhysX/ArticulationJointBus.h>
#include <PhysX/Joint/PhysXJointRequestsBus.h>
#include <VehicleDynamics/Utilities.h>

namespace ROS2::VehicleDynamics
{
    void AckermannDriveModel::Reflect(AZ::ReflectContext* context)
    {
        AckermannModelLimits::Reflect(context);
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<AckermannDriveModel, DriveModel>()
                ->Version(1)
                ->Field("SteeringPID", &AckermannDriveModel::m_steeringPid)
                ->Field("Limits", &AckermannDriveModel::m_limits);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<AckermannDriveModel>("Simplified Drive Model", "Configuration of a simplified vehicle dynamics drive model")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &AckermannDriveModel::m_steeringPid,
                        "Steering PID",
                        "Configuration of steering PID controller")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &AckermannDriveModel::m_limits, "Vehicle Limits", "Limits");
            }
        }
    }

    AckermannDriveModel::AckermannDriveModel(const AckermannModelLimits& limits, const ROS2::Controllers::PidConfiguration& steeringPid)
        : m_limits(limits)
        , m_steeringPid(steeringPid)
    {
    }

    void AckermannDriveModel::Activate(const VehicleConfiguration& vehicleConfig)
    {
        m_driveWheelsData.clear();
        m_steeringData.clear();
        m_vehicleConfiguration = vehicleConfig;
        m_steeringPid.InitializePid();
    }

    void AckermannDriveModel::ApplyState(const VehicleInputs& inputs, AZ::u64 deltaTimeNs)
    {
        if (m_driveWheelsData.empty())
        {
            m_driveWheelsData = VehicleDynamics::Utilities::GetAllDriveWheelsData(m_vehicleConfiguration);
        }

        if (m_steeringData.empty())
        {
            m_steeringData = VehicleDynamics::Utilities::GetAllSteeringEntitiesData(m_vehicleConfiguration);
        }
        const auto jointPositions = inputs.m_jointRequestedPosition;
        const float steering = jointPositions.empty() ? 0 : jointPositions.front();
        ApplySteering(steering, deltaTimeNs);
        ApplySpeed(inputs.m_speed.GetX(), deltaTimeNs);
    }

    void AckermannDriveModel::ApplyWheelSteering(SteeringDynamicsData& wheelData, float steering, double deltaTimeNs)
    {
        const auto& steeringEntity = wheelData.m_steeringEntity;
        const auto& jointComponent = wheelData.m_steeringJoint;

        auto id = AZ::EntityComponentIdPair(steeringEntity, jointComponent);

        if (wheelData.m_isArticulation)
        {
            PhysX::ArticulationJointRequestBus::Event(
                steeringEntity,
                [&](PhysX::ArticulationJointRequests* joint)
                {
                    double currentSteeringAngle = joint->GetJointPosition(wheelData.m_axis);
                    const double pidCommand = m_steeringPid.ComputeCommand(steering - currentSteeringAngle, deltaTimeNs);
                    joint->SetDriveTargetVelocity(wheelData.m_axis, pidCommand);
                });
        }
        else
        {
            PhysX::JointRequestBus::Event(
                id,
                [&](PhysX::JointRequests* joint)
                {
                    double currentSteeringAngle = joint->GetPosition();
                    const double pidCommand = m_steeringPid.ComputeCommand(steering - currentSteeringAngle, deltaTimeNs);
                    joint->SetVelocity(pidCommand);
                });
        }
    }

    void AckermannDriveModel::ApplySteering(float steering, AZ::u64 deltaTimeNs)
    {
        if (m_disabled)
        {
            return;
        }
        if (m_steeringData.empty())
        {
            AZ_Warning("ApplySteering", false, "Cannot apply steering since no steering elements are defined in the model");
            return;
        }

        auto innerSteering = AZ::Atan2(
            (m_vehicleConfiguration.m_wheelbase * tan(steering)),
            (m_vehicleConfiguration.m_wheelbase - 0.5 * m_vehicleConfiguration.m_track * tan(steering)));
        auto outerSteering = AZ::Atan2(
            (m_vehicleConfiguration.m_wheelbase * tan(steering)),
            (m_vehicleConfiguration.m_wheelbase + 0.5 * m_vehicleConfiguration.m_track * tan(steering)));

        ApplyWheelSteering(m_steeringData.front(), innerSteering, deltaTimeNs);
        ApplyWheelSteering(m_steeringData.back(), outerSteering, deltaTimeNs);
    }

    void AckermannDriveModel::ApplySpeed(float speed, AZ::u64 deltaTimeNs)
    {
        if (m_disabled)
        {
            return;
        }
        const float acceleration = m_limits.GetLinearAcceleration();
        const float maxSpeed = m_limits.GetLinearSpeedLimit();
        m_speedCommand = Utilities::ComputeRampVelocity(speed, m_speedCommand, deltaTimeNs, acceleration, maxSpeed);

        if (m_driveWheelsData.empty())
        {
            AZ_Warning("ApplySpeed", false, "Cannot apply speed since no driving wheels are defined in the model");
            return;
        }

        for (const auto& wheelData : m_driveWheelsData)
        {
            float wheelRadius = wheelData.m_wheelRadius;
            AZ_Assert(wheelRadius != 0, "wheelRadius must be non-zero");
            auto desiredAngularSpeedX = (m_speedCommand / wheelRadius);
            VehicleDynamics::Utilities::SetWheelRotationSpeed(wheelData, desiredAngularSpeedX);
        }
    }

    const VehicleModelLimits* AckermannDriveModel::GetVehicleLimitPtr() const
    {
        return &m_limits;
    }

    AZStd::pair<AZ::Vector3, AZ::Vector3> AckermannDriveModel::GetVelocityFromModel()
    {
        AZ_Warning("ApplySpeed", false, "GetVelocityFromModel is not implemented for AckermannDriveModel");
        return AZStd::pair<AZ::Vector3, AZ::Vector3>{ AZ::Vector3::CreateZero(), AZ::Vector3::CreateZero() };
    }

} // namespace ROS2::VehicleDynamics
