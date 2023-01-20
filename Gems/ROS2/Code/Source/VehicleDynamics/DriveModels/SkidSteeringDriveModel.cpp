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

    void SkidSteeringDriveModel::ApplyState(const VehicleInputs& inputs, uint64_t deltaTimeNs)
    {
        float angular_speed = inputs.m_angularRates.GetZ();
        float linear_speed = inputs.m_speed.GetX();
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
            }
            AZ_Warning("SkidSteeringDriveModel", driveAxesCount == 0, "Skid steering model does not have any drive wheels.");
        }

        for (size_t axleCount = 0; axleCount < m_config.m_axles.size(); axleCount++)
        {
            const auto& axle = m_config.m_axles[axleCount];
            const auto wheelCount = axle.m_axleWheels.size();
            if (!axle.m_isDrive)
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
                float wh = normalizedWheelId * m_config.m_wheelbase;
                float vh = (linear_speed + angular_speed * wh) / axle.m_wheelRadius;
                PhysX::JointRequestBus::Event(hingePtr->second, &PhysX::JointRequests::SetVelocity, vh);
            }
        }
    }

    VehicleModelLimits const* SkidSteeringDriveModel::GetVehicleLimitPtr() const
    {
        return &m_limits;
    }

} // namespace ROS2::VehicleDynamics
