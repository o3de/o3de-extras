/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ROS2Sensors/ROS2SensorsTypeIds.h"
#include <ROS2/Sensor/Events/PhysicsBasedSource.h>
#include <ROS2/Sensor/Events/TickBasedSource.h>
#include <ROS2Sensors/Sensor/ROS2SensorComponentBase.h>
#include <ROS2Sensors/Sensor/SensorHelper.h>

namespace ROS2
{

    AZStd::vector<AZ::EntityComponentIdPair> GetSensorsForEntity(const AZ::EntityId& entityId)
    {
        AZStd::vector<AZ::EntityComponentIdPair> sensors;
        if (AZ::Entity* entity = AZ::Interface<AZ::ComponentApplicationRequests>::Get()->FindEntity(entityId))
        {
            auto components = entity->GetComponents();
            for (const auto* component : components)
            {
                AZ_Assert(component, "Component not found");
                if (IsComponentROS2Sensor(component))
                {
                    sensors.push_back(AZ::EntityComponentIdPair(entityId, component->GetId()));
                }
            }
        }
        else
        {
            AZ_Warning("SensorHelpers", false, "Entity not found");
        }
        return sensors;
    }

    AZStd::vector<AZ::EntityComponentIdPair> GetSensorsOfType(const AZ::EntityId& entityId, const AZ::Uuid& sensorType)
    {
        AZStd::vector<AZ::EntityComponentIdPair> sensors;
        if (AZ::Entity* entity = AZ::Interface<AZ::ComponentApplicationRequests>::Get()->FindEntity(entityId))
        {
            auto components = entity->GetComponents();
            for (const auto* component : components)
            {
                AZ_Assert(component, "Component not found");
                if (component->RTTI_IsTypeOf(sensorType))
                {
                    sensors.push_back(AZ::EntityComponentIdPair(entityId, component->GetId()));
                }
            }
        }
        else
        {
            AZ_Warning("SensorHelpers", false, "Entity not found");
        }
        return sensors;
    }

    bool IsComponentROS2Sensor(const AZ::Component* component)
    {
        // In ROS2Sensors gem we have at this moment two types of base classes for sensors, we need to check if the component is derived
        // from one of them. If we add more base classes for sensors in the future, we need to update this function.
        // if (azrtti_cast<const ROS2::ROS2SensorComponentBase<ROS2::TickBasedSource>*>(component))
        // {
        //     return true;
        // }
        // if (azrtti_cast<const ROS2::ROS2SensorComponentBase<ROS2::PhysicsBasedSource>*>(component))
        // {
        //     return true;
        // }
        if (component->GetUnderlyingComponentType() == AZ::TypeId(ROS2Sensors::ROS2SensorComponentBaseTypeId))
        {
            return true;
        }

        return false;
    }
}; // namespace ROS2
