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

#include <ROS2/RobotImporter/SDFormatSensorImporterHook.h>

#include <sdf/Link.hh>
#include <sdf/Sensor.hh>

namespace ROS2
{
    void AddSensor(AZ::EntityId entityId, const sdf::Sensor* sensor)
    {
        AZ::Entity* entity = AzToolsFramework::GetEntityById(entityId);
        auto serializeContext = AZ::Interface<AZ::ComponentApplicationRequests>::Get()->GetSerializeContext();
        serializeContext->EnumerateAll(
            [&](const AZ::SerializeContext::ClassData* classData, const AZ::Uuid& typeId) -> bool
            {
                auto* attribute = AZ::FindAttribute(AZ::Crc32("SDFormatSensorImporter"), classData->m_attributes);
                if (attribute == nullptr)
                {
                    return true;
                }

                AZ::AttributeReader reader(nullptr, attribute);
                SDFormat::SensorImporterHook sensorHook;
                if (reader.Read<SDFormat::SensorImporterHook>(sensorHook))
                {
                    for (const auto& t : sensorHook.m_sensorTypes)
                    {
                        if (sensor->Type() == t)
                        {
                            sensorHook.m_sdfSensorToComponentCallback(*entity, *sensor);
                            return false;
                        }
                    }
                }

                return true;
            });
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
