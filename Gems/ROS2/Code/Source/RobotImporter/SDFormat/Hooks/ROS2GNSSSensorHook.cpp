/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <GNSS/ROS2GNSSSensorComponent.h>
#include <ROS2/Frame/ROS2FrameEditorComponent.h>
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
        importerHook.m_supportedPluginParams = AZStd::unordered_set<AZStd::string>{ ">ros>remapping", ">ros>argument", ">ros>frame_name",
                                                                                    ">ros>namespace", ">frameName",    ">robotNamespace" };
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

            const auto gnssPlugins = sdfSensor.Plugins();
            HooksUtils::PluginParams gnssPluginParams =
                gnssPlugins.empty() ? HooksUtils::PluginParams() : HooksUtils::GetPluginParams(gnssPlugins[0]);

            // setting gnss topic
            AZStd::string messageTopic = "gnss";
            if (gnssPluginParams.contains("out"))
            {
                messageTopic = gnssPluginParams["out"];
            }
            else if (gnssPluginParams.contains("topicName"))
            {
                messageTopic = HooksUtils::PluginParser::LastOnPath(gnssPluginParams["topicName"]);
            }

            HooksUtils::AddTopicConfiguration(sensorConfiguration, messageTopic, messageType, messageType);

            // Create required components
            auto frameComponent = HooksUtils::CreateComponent<ROS2FrameEditorComponent>(entity);
            HooksUtils::ConfigureFrame(frameComponent, gnssPluginParams);

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
