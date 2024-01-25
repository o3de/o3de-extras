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

#include <sdf/Link.hh>
#include <sdf/Sensor.hh>

namespace ROS2
{
    void AddSensor(AZ::EntityId entityId, const sdf::Sensor* sensor)
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
                    AZ::Entity* entity = AzToolsFramework::GetEntityById(entityId);
                    hook.m_sdfSensorToComponentCallback(*entity, *sensor);
                    return;
                }
            }
            AZ_Warning(
                "SensorMaker", false, "Cannot find a hook for %s sensor (type %s)", sensor->Name().c_str(), sensor->TypeStr().c_str());
            return;
        }

        for (const auto& sp : sensorPlugins)
        {
            bool currentAdded = false;
            ROS2::SDFormat::SensorImporterHook* defaultHook = nullptr;
            for (auto& hook : sensorHooks)
            {
                if (hook.m_sensorTypes.contains(sensor->Type()))
                {
                    if (hook.m_pluginNames.contains(sp.Filename().c_str()))
                    {
                        AZ::Entity* entity = AzToolsFramework::GetEntityById(entityId);
                        hook.m_sdfSensorToComponentCallback(*entity, *sensor);
                        currentAdded = true;
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
                currentAdded,
                "Cannot find a hook for %s sensor (type %s) with plugin %s",
                sensor->Name().c_str(),
                sensor->TypeStr().c_str(),
                sp.Filename().c_str());
            if (currentAdded == false && defaultHook != nullptr)
            {
                AZ::Entity* entity = AzToolsFramework::GetEntityById(entityId);
                defaultHook->m_sdfSensorToComponentCallback(*entity, *sensor);
                currentAdded = true;
                AZ_Warning(
                    "SensorMaker", false, "Default hook for %s sensor (type %s) used.", sensor->Name().c_str(), sensor->TypeStr().c_str());
            }
        }
    }

    void SensorsMaker::AddSensors(const sdf::Model& model, const sdf::Link* link, AZ::EntityId entityId) const
    {
        for (size_t si = 0; si < link->SensorCount(); ++si)
        {
            const auto* sensor = link->SensorByIndex(si);
            AddSensor(entityId, sensor);
        }
    }
} // namespace ROS2
