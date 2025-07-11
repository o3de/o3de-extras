#
# Copyright (c) Contributors to the Open 3D Engine Project.
# For complete copyright and license terms please see the LICENSE at the root of this distribution.
#
# SPDX-License-Identifier: Apache-2.0 OR MIT
#
#

set(FILES
    Source/ROS2SensorsModuleInterface.cpp
    Source/ROS2SensorsModuleInterface.h
    Source/Clients/ROS2SensorsSystemComponent.cpp
    Source/Clients/ROS2SensorsSystemComponent.h
    Source/Camera/PostProcessing/ROS2ImageEncodingConversionComponent.cpp
    Source/Camera/PostProcessing/ROS2ImageEncodingConversionComponent.h
    Source/Camera/CameraPublishers.cpp
    Source/Camera/CameraPublishers.h
    Source/Camera/CameraSensor.cpp
    Source/Camera/CameraSensor.h
    Source/Camera/CameraSensorDescription.cpp
    Source/Camera/CameraSensorDescription.h
    Source/Camera/CameraSensorConfiguration.cpp
    Source/Camera/CameraUtilities.cpp
    Source/Camera/CameraUtilities.h
    Source/Camera/ROS2CameraSensorComponent.cpp
    Source/Camera/ROS2CameraSensorComponent.h
    Source/Camera/ROS2CameraSystemComponent.cpp
    Source/Camera/ROS2CameraSystemComponent.h
    Source/GNSS/ROS2GNSSSensorComponent.cpp
    Source/GNSS/ROS2GNSSSensorComponent.h
    Source/Imu/ImuSensorConfiguration.cpp
    Source/Imu/ROS2ImuSensorComponent.cpp
    Source/Imu/ROS2ImuSensorComponent.h
    Source/Lidar/ClassSegmentationConfigurationComponent.cpp
    Source/Lidar/ClassSegmentationConfigurationComponent.h
    Source/Lidar/LidarCore.cpp
    Source/Lidar/LidarCore.h
    Source/Lidar/LidarRaycaster.cpp
    Source/Lidar/LidarRaycaster.h
    Source/Lidar/LidarRegistrarSystemComponent.cpp
    Source/Lidar/LidarRegistrarSystemComponent.h
    Source/Lidar/LidarSystem.cpp
    Source/Lidar/LidarSystem.h
    Source/Lidar/LidarTemplate.cpp
    Source/Lidar/PointCloudMessageBuilder.cpp
    Source/Lidar/PointCloudMessageBuilder.h
    Source/Lidar/ROS2Lidar2DSensorComponent.cpp
    Source/Lidar/ROS2Lidar2DSensorComponent.h
    Source/Lidar/ROS2LidarSensorComponent.cpp
    Source/Lidar/ROS2LidarSensorComponent.h
    Source/Odometry/ROS2OdometrySensorComponent.cpp
    Source/Odometry/ROS2OdometrySensorComponent.h
)

# optional, legacy features compilation
if (WITH_GAZEBO_MSGS)
        list(APPEND FILES
                Source/ContactSensor/ROS2ContactSensorComponent.cpp
                Source/ContactSensor/ROS2ContactSensorComponent.h
        )
endif ()
