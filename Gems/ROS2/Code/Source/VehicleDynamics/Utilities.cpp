/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "Utilities.h"
#include "Source/ArticulationLinkComponent.h"
#include "WheelControllerComponent.h"
#include <AzCore/Component/ComponentApplicationBus.h>
#include <AzCore/Component/Entity.h>
#include <AzCore/std/string/string.h>
#include <HingeJointComponent.h>
#include <PhysX/Joint/PhysXJointRequestsBus.h>
#include <Utilities/ArticulationsUtilities.h>

namespace ROS2::VehicleDynamics::Utilities
{
    AxleConfiguration Create2WheelAxle(
        AZ::EntityId leftWheel, AZ::EntityId rightWheel, AZStd::string tag, float wheelRadius, bool steering, bool drive)
    {
        VehicleDynamics::AxleConfiguration axleConfiguration;
        axleConfiguration.m_axleWheels.push_back(leftWheel);
        axleConfiguration.m_axleWheels.push_back(rightWheel);
        axleConfiguration.m_axleTag = AZStd::move(tag);
        axleConfiguration.m_isSteering = steering;
        axleConfiguration.m_isDrive = drive;
        axleConfiguration.m_wheelRadius = wheelRadius;
        return axleConfiguration;
    }

    AxleConfiguration CreateFrontSteerAndDriveAxle(AZ::EntityId leftWheel, AZ::EntityId rightWheel, float wheelRadius)
    {
        return Create2WheelAxle(leftWheel, rightWheel, "Front", wheelRadius, true, true);
    }

    AxleConfiguration CreateRearDriveAxle(AZ::EntityId leftWheel, AZ::EntityId rightWheel, float wheelRadius)
    {
        return Create2WheelAxle(leftWheel, rightWheel, "Rear", wheelRadius, false, true);
    }

    AZStd::vector<VehicleDynamics::SteeringDynamicsData> GetAllSteeringEntitiesData(const VehicleConfiguration& vehicleConfig)
    {
        AZStd::vector<VehicleDynamics::SteeringDynamicsData> steeringEntitiesAndAxis;
        for (const auto& axle : vehicleConfig.m_axles)
        {
            if (!axle.m_isSteering)
            { // Only steering axles will have steering entities
                continue;
            }

            for (const auto& wheel : axle.m_axleWheels)
            {
                if (!wheel.IsValid())
                {
                    AZ_Warning("GetAllSteeringEntitiesData", false, "Wheel entity in axle %s is invalid, ignoring", axle.m_axleTag.c_str());
                    continue;
                }
                AZ::Entity* wheelEntity = nullptr;
                AZ::ComponentApplicationBus::BroadcastResult(wheelEntity, &AZ::ComponentApplicationRequests::FindEntity, wheel);
                if (!wheelEntity)
                {
                    AZ_Warning("GetAllSteeringEntitiesData", false, "Wheel entity in axle %s is null, ignoring", axle.m_axleTag.c_str());
                    continue;
                }

                auto* controllerComponent = wheelEntity->FindComponent<WheelControllerComponent>();
                if (!controllerComponent)
                {
                    AZ_Warning(
                        "GetAllSteeringEntitiesData",
                        false,
                        "Missing a WheelController in wheel entity %s, ignoring",
                        wheel.ToString().c_str());
                    continue;
                }

                AZ::EntityId steeringEntity = controllerComponent->m_steeringEntity;
                if (!steeringEntity.IsValid())
                {
                    AZ_Warning(
                        "GetAllSteeringEntitiesData",
                        false,
                        "Steering entity specified for WheelController in entity %s is invalid, ignoring",
                        wheel.ToString().c_str());
                    continue;
                }

                const float steeringScale = controllerComponent->m_steeringScale;
                AZ::Entity* steeringEntityptr{ nullptr };
                AZ::ComponentApplicationBus::BroadcastResult(
                    steeringEntityptr, &AZ::ComponentApplicationRequests::FindEntity, steeringEntity);
                AZ_Assert(steeringEntityptr, "Cannot find a steering entity for %s", steeringEntity.ToString().c_str());

                PhysX::HingeJointComponent* hingeComponent{ nullptr };
                hingeComponent = steeringEntityptr->FindComponent<PhysX::HingeJointComponent>();
                PhysX::ArticulationLinkComponent* articulation{ nullptr };
                articulation = steeringEntityptr->FindComponent<PhysX::ArticulationLinkComponent>();

                if (!hingeComponent && !articulation)
                {
                    AZ_Warning(
                        "GetAllSteeringEntitiesData",
                        false,
                        "Steering entity specified for WheelController in entity %s does not have either a HingeJointComponent or an "
                        "ArticulationLinkComponent, ignoring",
                        wheel.ToString().c_str());
                    continue;
                }

                if (articulation)
                {
                    if (articulation->m_config.m_articulationJointType != PhysX::ArticulationJointType::Hinge)
                    {
                        AZ_Warning(
                            "GetAllSteeringEntitiesData",
                            false,
                            "Steering entity specified for WheelController in entity %s has an Articulation Link, but it's not a hinge "
                            "joint, ignoring",
                            wheel.ToString().c_str());
                        continue;
                    }
                }

                VehicleDynamics::SteeringDynamicsData steeringData;
                steeringData.m_steeringScale = steeringScale;
                steeringData.m_steeringEntity = steeringEntity;
                steeringData.m_isArticulation = articulation;
                if (articulation)
                {
                    steeringData.m_steeringJoint = articulation->GetId();
                    const bool hasFreeAxis = Utils::TryGetFreeArticulationAxis(steeringData.m_steeringEntity, steeringData.m_axis);

                    AZ_Error("VehicleDynamics::Utilities", hasFreeAxis, "Articulation steering has no free axis somehow");
                }
                else
                {
                    steeringData.m_steeringJoint = hingeComponent->GetId();
                }
                steeringEntitiesAndAxis.push_back(steeringData);
            }
        }
        return steeringEntitiesAndAxis;
    }

    AZStd::vector<VehicleDynamics::WheelDynamicsData> GetAllDriveWheelsData(const VehicleConfiguration& vehicleConfig)
    {
        AZStd::vector<VehicleDynamics::WheelDynamicsData> driveWheelEntities;
        for (const AxleConfiguration& axle : vehicleConfig.m_axles)
        {
            if (!axle.m_isDrive)
            { // Get only drive wheels, which are attached to a drive axle
                continue;
            }

            for (const AZ::EntityId& wheel : axle.m_axleWheels)
            {
                if (!wheel.IsValid())
                {
                    AZ_Warning("GetAllSteeringEntitiesData", false, "Wheel entity in axle %s is invalid, ignoring", axle.m_axleTag.c_str());
                    continue;
                }
                AZ::Entity* wheelEntity = nullptr;
                AZ::ComponentApplicationBus::BroadcastResult(wheelEntity, &AZ::ComponentApplicationRequests::FindEntity, wheel);
                if (!wheelEntity)
                {
                    AZ_Warning("GetAllSteeringEntitiesData", false, "Wheel entity in axle %s is null, ignoring", axle.m_axleTag.c_str());
                    continue;
                }
                auto* controllerComponent = wheelEntity->FindComponent<WheelControllerComponent>();
                if (!controllerComponent)
                {
                    AZ_Warning(
                        "GetAllDriveWheelsData",
                        false,
                        "Wheel entity for axle %s is missing a WheelController component, ignoring",
                        axle.m_axleTag.c_str());
                    continue;
                }

                PhysX::HingeJointComponent* hingeComponent{ nullptr };
                hingeComponent = wheelEntity->FindComponent<PhysX::HingeJointComponent>();

                PhysX::ArticulationLinkComponent* articulation{ nullptr };
                articulation = wheelEntity->FindComponent<PhysX::ArticulationLinkComponent>();

                if (!hingeComponent && !articulation)
                {
                    AZ_Warning(
                        "GetAllDriveWheelsData",
                        false,
                        "Wheel entity for axle %s does not have a HingeJointComponent nor an ArticulationLinkComponent, ignoring",
                        axle.m_axleTag.c_str());
                    continue;
                }

                if (articulation)
                {
                    if (articulation->m_config.m_articulationJointType != PhysX::ArticulationJointType::Hinge)
                    {
                        AZ_Warning(
                            "GetAllDriveWheelsData",
                            false,
                            "Wheel entity for axle %s has an Articulation Link, but it's not a hinge joint, ignoring",
                            wheel.ToString().c_str());
                        continue;
                    }
                }

                VehicleDynamics::WheelDynamicsData wheelData;
                wheelData.m_isArticulation = articulation;
                wheelData.m_wheelEntity = wheel;
                if (articulation)
                {
                    wheelData.m_wheelJoint = articulation->GetId();
                    const bool hasFreeAxis = Utils::TryGetFreeArticulationAxis(wheelData.m_wheelEntity, wheelData.m_axis);

                    AZ_Error("VehicleDynamics::Utilities", hasFreeAxis, "Articulation wheel has no free axis somehow");
                }
                else
                {
                    wheelData.m_wheelJoint = hingeComponent->GetId();
                }
                wheelData.m_wheelRadius = axle.m_wheelRadius;
                driveWheelEntities.push_back(wheelData);
            }
        }
        return driveWheelEntities;
    }

    AZ::EntityComponentIdPair GetWheelPhysxHinge(const AZ::EntityId wheelEntityId)
    {
        AZ::Entity* wheelEntity = nullptr;
        AZ::ComponentApplicationBus::BroadcastResult(wheelEntity, &AZ::ComponentApplicationRequests::FindEntity, wheelEntityId);
        if (!wheelEntity)
        {
            AZ_Warning("GetWheelDynamicData", false, "Entity %s was not found", wheelEntityId.ToString().c_str());
            return AZ::EntityComponentIdPair();
        }
        PhysX::HingeJointComponent* hingeComponent{ nullptr };
        hingeComponent = wheelEntity->FindComponent<PhysX::HingeJointComponent>();
        if (!hingeComponent)
        {
            AZ_Warning("GetWheelDynamicData", false, "Entity %s has no PhysX::HingeJointComponent", wheelEntityId.ToString().c_str());
            return AZ::EntityComponentIdPair();
        }
        return AZ::EntityComponentIdPair(wheelEntityId, hingeComponent->GetId());
    }

    float ComputeRampVelocity(float targetVelocty, float lastVelocity, AZ::u64 deltaTimeNs, float acceleration, float maxVelocity)
    {
        const float deltaTimeSec = 1e-9f * static_cast<float>(deltaTimeNs);
        const float deltaAcceleration = deltaTimeSec * acceleration;
        float commandVelocity = 0;
        if (AZStd::abs(lastVelocity - targetVelocty) < deltaAcceleration)
        {
            commandVelocity = targetVelocty;
        }
        else if (targetVelocty > lastVelocity)
        {
            commandVelocity = lastVelocity + deltaAcceleration;
        }
        else if (targetVelocty < lastVelocity)
        {
            commandVelocity = lastVelocity - deltaAcceleration;
        }
        return AZStd::clamp(commandVelocity, -maxVelocity, maxVelocity);
    }
} // namespace ROS2::VehicleDynamics::Utilities
