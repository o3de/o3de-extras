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
        m_wheelColumns.clear();
    }

    AZStd::tuple<VehicleDynamics::WheelControllerComponent*, AZ::Vector2, AZ::Vector3> SkidSteeringDriveModel::ProduceWheelColumn(
        int wheelNumber, const AxleConfiguration& axle, const int axisCount) const
    {
        const auto wheelCount = axle.m_axleWheels.size();
        const auto wheelEntityId = axle.m_axleWheels[wheelNumber];
        AZ::Entity* wheelEntityPtr = nullptr;
        AZ::ComponentApplicationBus::BroadcastResult(wheelEntityPtr, &AZ::ComponentApplicationRequests::FindEntity, wheelEntityId);
        AZ_Assert(wheelEntityPtr, "The wheelEntity should not be null here");
        auto* wheelControllerComponentPtr = Utils::GetGameOrEditorComponent<WheelControllerComponent>(wheelEntityPtr);
        auto wheelData = VehicleDynamics::Utilities::GetWheelData(wheelEntityId, axle.m_wheelRadius);
        AZ::Transform hingeTransform = Utilities::GetJointTransform(wheelData);
        if (wheelControllerComponentPtr)
        {
            const float normalizedWheelId = -1.f + 2.f * wheelNumber / (wheelCount - 1);
            AZ::Vector3 axis = hingeTransform.TransformVector({ 0.f, 1.f, 0.f });
            float wheelBase = normalizedWheelId * m_config.m_wheelbase;
            AZ_Assert(axle.m_wheelRadius != 0, "axle.m_wheelRadius must be non-zero");
            float dX = axle.m_wheelRadius / (wheelCount * axisCount);
            float dPhi = axle.m_wheelRadius / (wheelBase * axisCount);
            return { wheelControllerComponentPtr, AZ::Vector2{ dX, dPhi }, axis };
        }
        return { nullptr, AZ::Vector2::CreateZero(), AZ::Vector3::CreateZero() };
    }

    AZStd::pair<AZ::Vector3, AZ::Vector3> SkidSteeringDriveModel::GetVelocityFromModel()
    {
        // compute every wheel contribution to vehicle velocity
        if (m_wheelColumns.empty())
        {
            for (const auto& axle : m_config.m_axles)
            {
                const auto wheelCount = axle.m_axleWheels.size();
                if (!axle.m_isDrive || wheelCount < 1)
                {
                    continue;
                }
                for (size_t wheelId = 0; wheelId < wheelCount; wheelId++)
                {
                    const auto column = ProduceWheelColumn(wheelId, axle, m_config.m_axles.size());
                    if (AZStd::get<0>(column))
                    {
                        m_wheelColumns.emplace_back(column);
                    }
                }
            }
        }

        //! accumulated contribution to vehicle's linear movements of every wheel
        float d_x = 0;

        //! accumulated contribution to vehicle's rotational movements of every wheel
        float d_fi = 0;

        // It is basically multiplication of matrix by a vector.
        for (auto& [wheel, column, axis] : m_wheelColumns)
        {
            const float omega = axis.Dot(wheel->GetRotationVelocity());
            d_x += omega * column.GetX();
            d_fi += omega * column.GetY();
        }

        return AZStd::pair<AZ::Vector3, AZ::Vector3>{ { d_x, 0, 0 }, { 0, 0, d_fi } };
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
                    auto wheelData = VehicleDynamics::Utilities::GetWheelData(wheel, axle.m_wheelRadius);
                    m_wheelsData[wheel] = wheelData;
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

        for (const auto& axle : m_config.m_axles)
        {
            const auto wheelCount = axle.m_axleWheels.size();
            if (!axle.m_isDrive || wheelCount < 1)
            {
                continue;
            }
            for (size_t wheelId = 0; wheelId < wheelCount; wheelId++)
            {
                const auto& wheelEntityId = axle.m_axleWheels[wheelId];
                float normalizedWheelId = -1.f + 2.f * wheelId / (wheelCount - 1);
                float wheelBase = normalizedWheelId * m_config.m_wheelbase / 2.f;
                AZ_Assert(axle.m_wheelRadius != 0, "axle.m_wheelRadius must be non-zero");
                float wheelRate = (m_currentLinearVelocity + m_currentAngularVelocity * wheelBase) / axle.m_wheelRadius;
                const auto& wheelData = m_wheelsData[wheelEntityId];
                VehicleDynamics::Utilities::SetWheelRotationSpeed(wheelData, wheelRate);
            }
        }
    }

    const VehicleModelLimits* SkidSteeringDriveModel::GetVehicleLimitPtr() const
    {
        return &m_limits;
    }

} // namespace ROS2::VehicleDynamics
