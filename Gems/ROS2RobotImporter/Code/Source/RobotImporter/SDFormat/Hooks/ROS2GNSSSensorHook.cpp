/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <AzCore/Math/Uuid.h>
#include <ROS2/ROS2EditorBus.h>
#include <ROS2/Sensor/Events/TickBasedSource.h>
#include <ROS2/Sensor/ROS2SensorComponentBase.h>
#include <ROS2Sensors/ROS2SensorsEditorBus.h>
#include <RobotImporter/SDFormat/ROS2SDFormatHooksUtils.h>
#include <RobotImporter/SDFormat/ROS2SensorHooks.h>

#include <sdf/NavSat.hh>
#include <sdf/Sensor.hh>

namespace ROS2RobotImporter::SDFormat
{
    SensorImporterHook ROS2SensorHooks::ROS2GNSSSensor()
    {
        SensorImporterHook importerHook;
        importerHook.m_sensorTypes = AZStd::unordered_set<sdf::SensorType>{ sdf::SensorType::NAVSAT };
        importerHook.m_supportedSensorParams = AZStd::unordered_set<AZStd::string>{ ">pose", ">update_rate", ">topic", ">visualize" };
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

            const auto gnssPluginParams = HooksUtils::GetPluginParams(sdfSensor.Plugins());
            const auto element = sdfSensor.Element();

            // Get base sensor configuration
            ROS2::SensorConfiguration sensorConfiguration;
            element->Get<bool>("visualize", sensorConfiguration.m_visualize, false);
            element->Get<float>("update_rate", sensorConfiguration.m_frequency, 10.0f);
            const AZStd::string messageTopic = HooksUtils::GetTopicName(gnssPluginParams, element, "gnss");
            const AZStd::string messageType = "sensor_msgs::msg::NavSatFix";
            HooksUtils::AddTopicConfiguration(sensorConfiguration, messageTopic, messageType, messageType);

            // Get frame configuration
            const auto frameConfiguration = HooksUtils::GetFrameConfiguration(gnssPluginParams);

            // Create GNSS component
            auto* ros2interface = ROS2::ROS2EditorInterface::Get();
            AZ_Assert(ros2interface, "ROS2EditorInterface not available in ROS2ImuSensorHook");
            auto* sensorsInterface = ROS2Sensors::ROS2SensorsEditorInterface::Get();
            AZ_Assert(sensorsInterface, "ROS2SensorsEditorInterface not available in ROS2GNSSSensorHook");
            if (ros2interface && sensorsInterface)
            {
                auto* ros2FrameComponent = ros2interface->CreateROS2FrameEditorComponent(entity, frameConfiguration);
                auto* sensor = sensorsInterface->CreateROS2GnssSensorComponent(entity, sensorConfiguration);
                if (ros2FrameComponent && sensor)
                {
                    return AZ::Success();
                }
            }

            return AZ::Failure(AZStd::string("Failed to create ROS 2 Imu Sensor component"));
        };

        return importerHook;
    }
} // namespace ROS2RobotImporter::SDFormat
