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

    void SkidSteeringDriveModel::Activate(const VehicleConfiguration& vehicleConfig)
    {
        m_config = vehicleConfig;
        m_wheelsData.clear();
    }

    void SkidSteeringDriveModel::ApplyState(const VehicleInputs& inputs, AZ::u64 deltaTimeNs)
    {
        if (m_disabled)
        {
            return;
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
        // cache PhysX Hinge component IDs
        if (m_wheelsData.empty())
        {
            int driveAxesCount = 0;
            for (const auto& axle : m_config.m_axles)
            {
                if (axle.m_isDrive)
                {
                    driveAxesCount++;
                }
                for (const auto& wheel : axle.m_axleWheels)
                {
                    auto hinge = VehicleDynamics::Utilities::GetWheelPhysxHinge(wheel);
                    m_wheelsData[wheel] = hinge;
                }
                AZ_Warning(
                    "SkidSteeringDriveModel",
                    axle.m_axleWheels.size() > 1,
                    "Axle %s has not enough wheels (%d)",
                    axle.m_axleTag.c_str(),
                    axle.m_axleWheels.size());
            }
            AZ_Warning("SkidSteeringDriveModel", driveAxesCount != 0, "Skid steering model does not have any drive wheels.");
        }

        for (size_t axleCount = 0; axleCount < m_config.m_axles.size(); axleCount++)
        {
            const auto& axle = m_config.m_axles[axleCount];
            const auto wheelCount = axle.m_axleWheels.size();
            if (!axle.m_isDrive || wheelCount < 1)
            {
                continue;
            }
            for (size_t wheelId = 0; wheelId < wheelCount; wheelId++)
            {
                const auto& wheel = axle.m_axleWheels[wheelId];
                auto hingePtr = m_wheelsData.find(wheel);
                if (hingePtr == m_wheelsData.end())
                {
                    continue;
                }
                float normalizedWheelId = -1.f + 2.f * wheelId / (wheelCount - 1);
                float wheelBase = normalizedWheelId * m_config.m_wheelbase;
                AZ_Assert(axle.m_wheelRadius != 0, "axle.m_wheelRadius must be non-zero");
                float wheelRate = (m_currentLinearVelocity + m_currentAngularVelocity * wheelBase) / axle.m_wheelRadius;
                PhysX::JointRequestBus::Event(hingePtr->second, &PhysX::JointRequests::SetVelocity, wheelRate);
            }
        }
    }

    const VehicleModelLimits* SkidSteeringDriveModel::GetVehicleLimitPtr() const
    {
        return &m_limits;
    }

} // namespace ROS2::VehicleDynamics
