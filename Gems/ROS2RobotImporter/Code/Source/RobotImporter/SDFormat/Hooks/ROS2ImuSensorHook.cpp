/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <AzCore/Math/Uuid.h>
#include <ROS2/Frame/ROS2FrameEditorComponent.h>
#include <ROS2/Sensor/Events/PhysicsBasedSource.h>
#include <ROS2/Sensor/ROS2SensorComponentBase.h>
#include <ROS2Sensors/Imu/ImuSensorConfiguration.h>
#include <ROS2Sensors/ROS2SensorsEditorBus.h>
#include <RobotImporter/SDFormat/ROS2SDFormatHooksUtils.h>
#include <RobotImporter/SDFormat/ROS2SensorHooks.h>
#include <Source/EditorArticulationLinkComponent.h>

#include <sdf/Imu.hh>
#include <sdf/Sensor.hh>

namespace ROS2RobotImporter::SDFormat
{
    SensorImporterHook ROS2SensorHooks::ROS2ImuSensor()
    {
        SensorImporterHook importerHook;
        importerHook.m_sensorTypes = AZStd::unordered_set<sdf::SensorType>{ sdf::SensorType::IMU };
        importerHook.m_supportedSensorParams = AZStd::unordered_set<AZStd::string>{ ">pose",
                                                                                    ">update_rate",
                                                                                    ">imu>angular_velocity>x>noise>mean",
                                                                                    ">imu>angular_velocity>x>noise>stddev",
                                                                                    ">imu>angular_velocity>y>noise>mean",
                                                                                    ">imu>angular_velocity>y>noise>stddev",
                                                                                    ">imu>angular_velocity>z>noise>mean",
                                                                                    ">imu>angular_velocity>z>noise>stddev",
                                                                                    ">imu>linear_acceleration>x>noise>mean",
                                                                                    ">imu>linear_acceleration>x>noise>stddev",
                                                                                    ">imu>linear_acceleration>y>noise>mean",
                                                                                    ">imu>linear_acceleration>y>noise>stddev",
                                                                                    ">imu>linear_acceleration>z>noise>mean",
                                                                                    ">imu>linear_acceleration>z>noise>stddev",
                                                                                    ">topic",
                                                                                    ">visualize" };
        importerHook.m_pluginNames = AZStd::unordered_set<AZStd::string>{ "libgazebo_ros_imu_sensor.so" };
        importerHook.m_supportedPluginParams =
            AZStd::unordered_set<AZStd::string>{ ">topicName",     ">ros>remapping", ">ros>argument",  ">ros>frame_name",
                                                 ">ros>namespace", ">frameName",     ">robotNamespace" };
        importerHook.m_sdfSensorToComponentCallback = [](AZ::Entity& entity,
                                                         const sdf::Sensor& sdfSensor) -> SensorImporterHook::ConvertSensorOutcome
        {
            auto* imuSensor = sdfSensor.ImuSensor();
            if (!imuSensor)
            {
                return AZ::Failure(AZStd::string("Failed to read parsed SDFormat data of %s imu sensor", sdfSensor.Name().c_str()));
            }

            ROS2Sensors::ImuSensorConfiguration imuConfiguration;
            const auto& angVelXNoise = imuSensor->AngularVelocityXNoise();
            const auto& angVelYNoise = imuSensor->AngularVelocityYNoise();
            const auto& angVelZNoise = imuSensor->AngularVelocityZNoise();
            if (angVelXNoise.Type() == sdf::NoiseType::GAUSSIAN && angVelYNoise.Type() == sdf::NoiseType::GAUSSIAN &&
                angVelZNoise.Type() == sdf::NoiseType::GAUSSIAN)
            {
                if (angVelXNoise.Mean() == 0.0 && angVelYNoise.Mean() == 0.0 && angVelZNoise.Mean() == 0.0)
                {
                    imuConfiguration.m_angularVelocityVariance = AZ::Vector3(
                        static_cast<float>(angVelXNoise.StdDev() * angVelXNoise.StdDev()),
                        static_cast<float>(angVelYNoise.StdDev() * angVelYNoise.StdDev()),
                        static_cast<float>(angVelZNoise.StdDev() * angVelZNoise.StdDev()));
                }
            }

            const auto& linAccXNoise = imuSensor->LinearAccelerationXNoise();
            const auto& linAccYNoise = imuSensor->LinearAccelerationYNoise();
            const auto& linAccZNoise = imuSensor->LinearAccelerationZNoise();
            if (linAccXNoise.Type() == sdf::NoiseType::GAUSSIAN && linAccYNoise.Type() == sdf::NoiseType::GAUSSIAN &&
                linAccZNoise.Type() == sdf::NoiseType::GAUSSIAN)
            {
                if (linAccXNoise.Mean() == 0.0 && linAccYNoise.Mean() == 0.0 && linAccZNoise.Mean() == 0.0)
                {
                    imuConfiguration.m_linearAccelerationVariance = AZ::Vector3(
                        static_cast<float>(linAccXNoise.StdDev() * linAccXNoise.StdDev()),
                        static_cast<float>(linAccYNoise.StdDev() * linAccYNoise.StdDev()),
                        static_cast<float>(linAccZNoise.StdDev() * linAccZNoise.StdDev()));
                }
            }

            const auto imuPluginParams = HooksUtils::GetPluginParams(sdfSensor.Plugins());
            const auto element = sdfSensor.Element();

            // Get base sensor configuration
            ROS2::SensorConfiguration sensorConfiguration;
            element->Get<bool>("visualize", sensorConfiguration.m_visualize, false);
            element->Get<float>("update_rate", sensorConfiguration.m_frequency, 10.0f);
            const AZStd::string messageTopic = HooksUtils::GetTopicName(imuPluginParams, element, "imu");
            const AZStd::string messageType = "sensor_msgs::msg::Imu";
            HooksUtils::AddTopicConfiguration(sensorConfiguration, messageTopic, messageType, messageType);

            // Get frame configuration
            const auto frameConfiguration = HooksUtils::GetFrameConfiguration(imuPluginParams);

            // Create required components
            HooksUtils::CreateComponent<ROS2::ROS2FrameEditorComponent>(entity, frameConfiguration);
            HooksUtils::CreateComponent<PhysX::EditorArticulationLinkComponent>(entity);

            // Create Imu component
            auto interface = ROS2Sensors::ROS2SensorsEditorInterface::Get();
            AZ_Warning("ROS2RobotImporter", interface, "ROS2SensorsInterface is not available. Cannot create Imu sensor component.");
            if (interface)
            {
                if (auto* sensor = interface->CreateROS2ImuSensorComponent(entity, sensorConfiguration, imuConfiguration))
                {
                    return AZ::Success();
                }
            }

            return AZ::Failure(AZStd::string("Failed to create ROS 2 Imu Sensor component"));
        };

        return importerHook;
    }
} // namespace ROS2RobotImporter::SDFormat
