/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "SkidSteeringDriveModel.h"
#include <AzCore/Component/TransformBus.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzFramework/Physics/RigidBodyBus.h>
#include <HingeJointComponent.h>
#include <PhysX/Joint/PhysXJointRequestsBus.h>
#include <ROS2/ROS2GemUtilities.h>
#include <VehicleDynamics/Utilities.h>

namespace ROS2::VehicleDynamics
{
    void SkidSteeringDriveModel::Reflect(AZ::ReflectContext* context)
    {
        SkidSteeringModelLimits::Reflect(context);
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<SkidSteeringDriveModel, DriveModel>()->Version(1)->Field("Limits", &SkidSteeringDriveModel::m_limits);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<SkidSteeringDriveModel>("Skid Steering Drive Model", "Configuration of a simplified vehicle dynamics drive model")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &SkidSteeringDriveModel::m_limits, "Vehicle Limits", "Limits");
            }
        }
    }

    SkidSteeringDriveModel::SkidSteeringDriveModel(const SkidSteeringModelLimits& limits)
        : m_limits(limits)
    {
    }

    void SkidSteeringDriveModel::Activate(const VehicleConfiguration& vehicleConfig)
    {
        m_config = vehicleConfig;
        m_wheelsData.clear();
        m_initialized = false;
    }

    void SkidSteeringDriveModel::ComputeWheelsData()
    {
        // Compute number of drive axles to scale the contribution of each wheel to vehicle's velocity
        int driveAxlesCount = 0;
        for (const auto& axle : m_config.m_axles)
        {
            if (axle.m_isDrive && axle.m_axleWheels.size() > 1)
            {
                driveAxlesCount++;
            }
            AZ_Warning(
                "SkidSteeringDriveModel",
                axle.m_axleWheels.size() > 1,
                "Axle %s has not enough wheels (%d)",
                axle.m_axleTag.c_str(),
                axle.m_axleWheels.size());
        }
        AZ_Warning("SkidSteeringDriveModel", driveAxlesCount != 0, "Skid steering model does not have any drive wheels.");

        // Find the contribution of each wheel to vehicle's velocity
        for (const auto& axle : m_config.m_axles)
        {
            const auto wheelCount = axle.m_axleWheels.size();
            if (!axle.m_isDrive || wheelCount <= 1)
            {
                continue;
            }
            for (size_t wheelId = 0; wheelId < wheelCount; wheelId++)
            {
                const auto data = ComputeSingleWheelData(wheelId, axle, driveAxlesCount);
                if (data.wheelControllerComponentPtr != nullptr)
                {
                    m_wheelsData.emplace_back(AZStd::move(data));
                }
            }
        }

        m_initialized = true;
    }

    SkidSteeringDriveModel::SkidSteeringWheelData SkidSteeringDriveModel::ComputeSingleWheelData(
        const int wheelId, const AxleConfiguration& axle, const int axlesCount) const
    {
        SkidSteeringDriveModel::SkidSteeringWheelData out;

        AZ_Assert(axle.m_wheelRadius != 0, "axle.m_wheelRadius must be non-zero");
        const auto wheelEntityId = axle.m_axleWheels[wheelId];
        AZ::Entity* wheelEntityPtr = nullptr;
        AZ::ComponentApplicationBus::BroadcastResult(wheelEntityPtr, &AZ::ComponentApplicationRequests::FindEntity, wheelEntityId);
        AZ_Assert(wheelEntityPtr, "The wheelEntity should not be null here");
        out.wheelControllerComponentPtr = Utils::GetGameOrEditorComponent<WheelControllerComponent>(wheelEntityPtr);
        out.wheelData = VehicleDynamics::Utilities::GetWheelData(wheelEntityId, axle.m_wheelRadius);
        if (out.wheelControllerComponentPtr)
        {
            const auto wheelsCount = axle.m_axleWheels.size();
            const float normalizedWheelId = -1.f + 2.f * wheelId / (wheelsCount - 1);
            out.wheelPosition = normalizedWheelId * (m_config.m_track / 2.f);
            out.dX = axle.m_wheelRadius / (wheelsCount * axlesCount);
            out.dPhi = axle.m_wheelRadius / (out.wheelPosition * axlesCount * axle.m_axleWheels.size());

            AZ::Transform hingeTransform = Utilities::GetJointTransform(out.wheelData);
            out.axis = hingeTransform.TransformVector({ 0.f, 1.f, 0.f });
        }

        return out;
    }

    AZStd::pair<AZ::Vector3, AZ::Vector3> SkidSteeringDriveModel::GetVelocityFromModel()
    {
        if (!m_initialized)
        {
            ComputeWheelsData();
        }

        float dX = 0; // accumulated contribution to vehicle's linear movements of every wheel
        float dPhi = 0; // accumulated contribution to vehicle's rotational movements of every wheel

        for (const auto& wheel : m_wheelsData)
        {
            const float omega = wheel.axis.Dot(wheel.wheelControllerComponentPtr->GetRotationVelocity());
            dX += omega * wheel.dX;
            dPhi += omega * wheel.dPhi;
        }

        return AZStd::pair<AZ::Vector3, AZ::Vector3>{ { dX, 0, 0 }, { 0, 0, dPhi } };
    }

    void SkidSteeringDriveModel::ApplyState(const VehicleInputs& inputs, AZ::u64 deltaTimeNs)
    {
        if (m_disabled)
        {
            return;
        }

        if (!m_initialized)
        {
            ComputeWheelsData();
        }

        const float angularTargetSpeed = inputs.m_angularRates.GetZ();
        const float linearTargetSpeed = inputs.m_speed.GetX();
        const float angularAcceleration = m_limits.GetAngularAcceleration();
        const float linearAcceleration = m_limits.GetLinearAcceleration();
        const float maxLinearVelocity = m_limits.GetLinearSpeedLimit();
        const float maxAngularVelocity = m_limits.GetAngularSpeedLimit();

        m_currentAngularVelocity = Utilities::ComputeRampVelocity(
            angularTargetSpeed, m_currentAngularVelocity, deltaTimeNs, angularAcceleration, maxAngularVelocity);
        m_currentLinearVelocity =
            Utilities::ComputeRampVelocity(linearTargetSpeed, m_currentLinearVelocity, deltaTimeNs, linearAcceleration, maxLinearVelocity);

        for (const auto& wheel : m_wheelsData)
        {
            const float wheelRate =
                (m_currentLinearVelocity + m_currentAngularVelocity * wheel.wheelPosition) / wheel.wheelData.m_wheelRadius;
            VehicleDynamics::Utilities::SetWheelRotationSpeed(wheel.wheelData, wheelRate);
        }
    }

    const VehicleModelLimits* SkidSteeringDriveModel::GetVehicleLimitPtr() const
    {
        return &m_limits;
    }

} // namespace ROS2::VehicleDynamics
