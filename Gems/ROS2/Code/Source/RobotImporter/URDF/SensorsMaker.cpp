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
#include <RobotImporter/SDFormat/ROS2SDFormatHooksUtils.h>
#include <RobotImporter/URDF/PrefabMakerUtils.h>
#include <RobotImporter/Utils/RobotImporterUtils.h>

#include <sdf/Link.hh>
#include <sdf/Sensor.hh>

namespace ROS2
{
    SensorsMaker::SensorHookCallOutcome SensorsMaker::CallSensorHook(
        AZ::EntityId entityId,
        const sdf::Sensor* sensor,
        const SDFormat::SensorImporterHook* hook,
        AZStd::vector<AZ::EntityId>& createdEntities)
    {
        // Since O3DE does not allow origin for sensors, we need to create a sub-entity and store sensor there
        AZStd::string subEntityName = sensor->Name().empty() ? sensor->TypeStr().c_str() : sensor->Name().c_str();
        subEntityName.append("_sensor");
        auto createEntityResult = PrefabMakerUtils::CreateEntity(entityId, subEntityName);
        if (!createEntityResult.IsSuccess())
        {
            return AZ::Failure(AZStd::string::format("Unable to create a sub-entity %s for sensor element\n", subEntityName.c_str()));
        }

        auto sensorEntityId = createEntityResult.GetValue();
        if (!sensorEntityId.IsValid())
        {
            return AZ::Failure(AZStd::string::format("Created sub-entity %s for sensor element is invalid\n", subEntityName.c_str()));
        }

        createdEntities.emplace_back(sensorEntityId);
        AZ::Entity* sensorEntity = AzToolsFramework::GetEntityById(sensorEntityId);
        const auto sensorResult = hook->m_sdfSensorToComponentCallback(*sensorEntity, *sensor);
        SDFormat::HooksUtils::SetSensorEntityTransform(*sensorEntity, *sensor);

        if (sensorResult.IsSuccess())
        {
            const auto sensorElement = sensor->Element();
            const auto& unsupportedSensorParams = Utils::SDFormat::GetUnsupportedParams(sensorElement, hook->m_supportedSensorParams);
            AZStd::string status;
            if (unsupportedSensorParams.empty())
            {
                status = AZStd::string::format("%s (type %s) created successfully", sensor->Name().c_str(), sensor->TypeStr().c_str());
            }
            else
            {
                status = AZStd::string::format(
                    "%s (type %s) created, %lu parameters not parsed: ",
                    sensor->Name().c_str(),
                    sensor->TypeStr().c_str(),
                    unsupportedSensorParams.size());
                for (const auto& up : unsupportedSensorParams)
                {
                    status.append("\n\t - " + up);
                }
            }
            m_status.emplace(AZStd::move(status));
        }
        else
        {
            const auto message = AZStd::string::format(
                "%s (type %s) not created: %s", sensor->Name().c_str(), sensor->TypeStr().c_str(), sensorResult.GetError().c_str());
            m_status.emplace(message);
            return AZ::Failure(message);
        }

        return AZ::Success();
    }

    SensorsMaker::SensorHookCallOutcome SensorsMaker::AddSensor(
        AZ::EntityId entityId, const sdf::Sensor* sensor, AZStd::vector<AZ::EntityId>& createdEntities)
    {
        SDFormat::SensorImporterHooksStorage sensorHooks;
        ROS2::RobotImporterRequestBus::BroadcastResult(sensorHooks, &ROS2::RobotImporterRequest::GetSensorHooks);

        const auto& sensorPlugins = sensor->Plugins();
        // Add sensor without plugins
        if (sensorPlugins.empty())
        {
            for (auto& hook : sensorHooks)
            {
                if (hook.m_sensorTypes.contains(sensor->Type()))
                {
                    return CallSensorHook(entityId, sensor, &hook, createdEntities);
                }
            }
            return AZ::Failure(
                AZStd::string::format("Cannot find a hook for %s sensor (type %s)", sensor->Name().c_str(), sensor->TypeStr().c_str()));
        }

        // Add sensor with one or more plugins
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
                        const auto outcome = CallSensorHook(entityId, sensor, &hook, createdEntities);
                        if (!outcome.IsSuccess())
                        {
                            return outcome;
                        }
                        sensorProcessed = true;
                        break;
                    }
                    else if (defaultHook == nullptr)
                    {
                        defaultHook = &hook;
                    }
                }
            }

            if (sensorProcessed == false && defaultHook != nullptr)
            {
                const auto outcome = CallSensorHook(entityId, sensor, defaultHook, createdEntities);
                if (!outcome.IsSuccess())
                {
                    return outcome;
                }
            }
        }

        return AZ::Success();
    }

    AZStd::vector<AZ::EntityId> SensorsMaker::AddSensors(const sdf::Model& model, const sdf::Link* link, AZ::EntityId entityId)
    {
        AZStd::vector<AZ::EntityId> createdEntities;
        for (size_t si = 0; si < link->SensorCount(); ++si)
        {
            const auto* sensor = link->SensorByIndex(si);
            AddSensor(entityId, sensor, createdEntities);
        }

        return createdEntities;
    }

    const AZStd::set<AZStd::string>& SensorsMaker::GetStatusMessages() const
    {
        return m_status;
    }
} // namespace ROS2
