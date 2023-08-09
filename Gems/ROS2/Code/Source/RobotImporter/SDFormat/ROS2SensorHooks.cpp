/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ROS2SensorHooks.h"

#include <Camera/CameraConstants.h>
#include <Camera/ROS2CameraSensorEditorComponent.h>
#include <GNSS/ROS2GNSSSensorComponent.h>
#include <Imu/ROS2ImuSensorComponent.h>
#include <Lidar/ROS2LidarSensorComponent.h>
#include <ROS2/Frame/ROS2FrameComponent.h>
#include <RobotImporter/Utils/RobotImporterUtils.h>
#include <Source/EditorStaticRigidBodyComponent.h>

#include <sdf/Camera.hh>
#include <sdf/NavSat.hh>

namespace ROS2::SDFormat
{
    namespace ROS2SensorHooks
    {
        void Utils::AddTopicConfiguration(
            SensorConfiguration& sensorConfig,
            const AZStd::string& topic,
            const AZStd::string& messageType,
            const AZStd::string& configName)
        {
            TopicConfiguration config;
            config.m_topic = topic;
            config.m_type = messageType;
            sensorConfig.m_publishersConfigurations.insert(AZStd::make_pair(configName, config));
        }
    } // namespace ROS2SensorHooks

} // namespace ROS2::SDFormat
