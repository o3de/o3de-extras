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
    bool AddSensor(AZ::EntityId entityId, const sdf::Sensor* sensor)
    {
        SDFormat::SensorImporterHooksStorage sensorHooks;
        ROS2::RobotImporterRequestBus::BroadcastResult(sensorHooks, &ROS2::RobotImporterRequest::GetSensorHooks);
        for (const auto& hook : sensorHooks)
        {
            if (hook.m_sensorTypes.contains(sensor->Type()))
            {
                AZ::Entity* entity = AzToolsFramework::GetEntityById(entityId);
                hook.m_sdfSensorToComponentCallback(*entity, *sensor);
                return true;
            }
        }

        return false;
    }

    void SensorsMaker::AddSensors(const sdf::Model& model, const sdf::Link* link, AZ::EntityId entityId) const
    {
        for (size_t si = 0; si < link->SensorCount(); ++si)
        {
            const auto* sensor = link->SensorByIndex(si);
            const bool success = AddSensor(entityId, sensor);
            AZ_Warning("SensorMaker", success, "Cannot find a sensor hook for sensor %d", sensor->Type());
        }
    }
} // namespace ROS2
