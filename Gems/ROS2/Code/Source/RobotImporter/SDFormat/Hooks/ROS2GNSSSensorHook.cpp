/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <GNSS/ROS2GNSSSensorComponent.h>
#include <RobotImporter/SDFormat/ROS2SDFormatHooksUtils.h>
#include <RobotImporter/SDFormat/ROS2SensorHooks.h>

#include <sdf/NavSat.hh>
#include <sdf/Sensor.hh>

namespace ROS2::SDFormat
{
    SensorImporterHook ROS2SensorHooks::ROS2GNSSSensor()
    {
        SensorImporterHook importerHook;
        importerHook.m_sensorTypes = AZStd::unordered_set<sdf::SensorType>{ sdf::SensorType::NAVSAT };
        importerHook.m_supportedSensorParams = AZStd::unordered_set<AZStd::string>{ ">pose", ">update_rate" };
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
            HooksUtils::AddTopicConfiguration(sensorConfiguration, "gnss", messageType, messageType);

            if (HooksUtils::CreateComponent<ROS2GNSSSensorComponent>(entity, sensorConfiguration))
            {
                return AZ::Success();
            }
            else
            {
                return AZ::Failure(AZStd::string("Failed to create ROS2 GNSS Sensor component"));
            }
        };

        return importerHook;
    }
} // namespace ROS2::SDFormat
