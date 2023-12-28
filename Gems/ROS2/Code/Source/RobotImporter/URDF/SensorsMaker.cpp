/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "SensorsMaker.h"

#include <AzCore/Component/EntityId.h>
#include <AzToolsFramework/Entity/EditorEntityHelpers.h>
#include <ROS2/RobotImporter/RobotImporterBus.h>
#include <ROS2/RobotImporter/SDFormatSensorImporterHook.h>
#include <RobotImporter/SDFormat/ROS2SDFormatHooksUtils.h>
#include <RobotImporter/URDF/PrefabMakerUtils.h>

#include <sdf/Link.hh>
#include <sdf/Sensor.hh>

namespace ROS2
{
    void SensorsMaker::ExecuteSensorHook(
        AZ::EntityId entityId,
        const sdf::Sensor* sensor,
        const ROS2::SDFormat::SensorImporterHook* hook,
        AZStd::vector<AZ::EntityId>& createdEntities)
    {
        // Since O3DE does not allow origin for sensors, we need to create a sub-entity and store sensor there
        AZStd::string subEntityName = sensor->Name().empty() ? sensor->TypeStr().c_str() : sensor->Name().c_str();
        subEntityName.append("_sensor");
        auto createEntityResult = PrefabMakerUtils::CreateEntity(entityId, subEntityName);
        if (!createEntityResult.IsSuccess())
        {
            AZ_Error("SensorMaker", false, "Unable to create a sub-entity %s for sensor element\n", subEntityName.c_str());
            return;
        }

        auto sensorEntityId = createEntityResult.GetValue();
        if (!sensorEntityId.IsValid())
        {
            AZ_Error("SensorMaker", false, "Created sub-entity %s for sensor element is invalid\n", subEntityName.c_str());
            return;
        }

        createdEntities.emplace_back(sensorEntityId);
        AZ::Entity* sensorEntity = AzToolsFramework::GetEntityById(sensorEntityId);
        hook->m_sdfSensorToComponentCallback(*sensorEntity, *sensor);
        SDFormat::HooksUtils::SetSensorEntityTransform(*sensorEntity, *sensor);
    }

    void SensorsMaker::AddSensor(AZ::EntityId entityId, const sdf::Sensor* sensor, AZStd::vector<AZ::EntityId>& createdEntities)
    {
        SDFormat::SensorImporterHooksStorage sensorHooks;
        ROS2::RobotImporterRequestBus::BroadcastResult(sensorHooks, &ROS2::RobotImporterRequest::GetSensorHooks);

        const auto& sensorPlugins = sensor->Plugins();
        if (sensorPlugins.empty())
        {
            for (auto& hook : sensorHooks)
            {
                if (hook.m_sensorTypes.contains(sensor->Type()))
                {
                    ExecuteSensorHook(entityId, sensor, &hook, createdEntities);
                    return;
                }
            }
            AZ_Warning(
                "SensorMaker", false, "Cannot find a hook for %s sensor (type %s)", sensor->Name().c_str(), sensor->TypeStr().c_str());
            return;
        }

        for (const auto& sp : sensorPlugins)
        {
            bool sensorProcessed = false;
            ROS2::SDFormat::SensorImporterHook* defaultHook = nullptr;
            for (auto& hook : sensorHooks)
            {
                if (hook.m_sensorTypes.contains(sensor->Type()))
                {
                    if (hook.m_pluginNames.contains(sp.Filename().c_str()))
                    {
                        ExecuteSensorHook(entityId, sensor, &hook, createdEntities);
                        sensorProcessed = true;
                        break;
                    }
                    else if (defaultHook == nullptr)
                    {
                        defaultHook = &hook;
                    }
                }
            }

            AZ_Warning(
                "SensorMaker",
                sensorProcessed,
                "Cannot find a hook for %s sensor (type %s) with plugin %s",
                sensor->Name().c_str(),
                sensor->TypeStr().c_str(),
                sp.Filename().c_str());
            if (sensorProcessed == false && defaultHook != nullptr)
            {
                ExecuteSensorHook(entityId, sensor, defaultHook, createdEntities);
                AZ_Warning(
                    "SensorMaker", false, "Default hook for %s sensor (type %s) used.", sensor->Name().c_str(), sensor->TypeStr().c_str());
            }
        }
    }

    AZStd::vector<AZ::EntityId> SensorsMaker::AddSensors(const sdf::Model& model, const sdf::Link* link, AZ::EntityId entityId) const
    {
        AZStd::vector<AZ::EntityId> createdEntities;
        for (size_t si = 0; si < link->SensorCount(); ++si)
        {
            const auto* sensor = link->SensorByIndex(si);
            AddSensor(entityId, sensor, createdEntities);
        }

        return createdEntities;
    }
} // namespace ROS2
