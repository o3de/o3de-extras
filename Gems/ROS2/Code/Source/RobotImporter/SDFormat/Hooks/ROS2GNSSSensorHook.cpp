/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <GNSS/ROS2GNSSSensorComponent.h>
#include <ROS2/ROS2GemUtilities.h>
#include <RobotImporter/SDFormat/ROS2SensorHooks.h>

#include <sdf/NavSat.hh>
#include <sdf/Sensor.hh>

namespace ROS2::SDFormat
{
    SensorImporterHook ROS2SensorHooks::ROS2GNSSSensor()
    {
        SensorImporterHook importerHook;
        importerHook.m_sensorTypes = AZStd::unordered_set<sdf::SensorType>{ sdf::SensorType::NAVSAT };
        importerHook.m_supportedSensorParams = AZStd::unordered_set<AZStd::string>{ ">update_rate" };
        importerHook.m_pluginNames = AZStd::unordered_set<AZStd::string>{ "libgazebo_ros_gps_sensor.so" };
        importerHook.m_supportedPluginParams = AZStd::unordered_set<AZStd::string>{};
        importerHook.m_sdfSensorToComponentCallback = [](AZ::Entity& entity,
                                                         const sdf::Sensor& sdfSensor) -> SensorImporterHook::ConvertSensorOutcome
        {
            if (!sdfSensor.NavSatSensor())
            {
                return AZ::Failure(AZStd::string("Failed to read parsed SDFormat data of %s NavSat sensor", sdfSensor.Name().c_str()));
            }

            SensorConfiguration sensorConfiguration;
            sensorConfiguration.m_frequency = sdfSensor.UpdateRate();
            const AZStd::string messageType = "sensor_msgs::msg::NavSatFix";
            Utils::AddTopicConfiguration(sensorConfiguration, "gnss", messageType, messageType);

            const auto& entityId = entity.GetId();
            const auto componentId = ROS2::Utils::CreateComponent(entityId, ROS2GNSSSensorComponent::TYPEINFO_Uuid());
            if (componentId)
            {
                auto* component = ROS2::Utils::GetGameOrEditorComponent<ROS2GNSSSensorComponent>(&entity);
                component->SetSensorConfiguration(sensorConfiguration);
            }
            else
            {
                return AZ::Failure(AZStd::string("Failed to create ROS2 GNSS Sensor component"));
            }

            return AZ::Success();
        };

        return importerHook;
    }
} // namespace ROS2::SDFormat
