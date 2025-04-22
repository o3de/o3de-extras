#
# Copyright (c) Contributors to the Open 3D Engine Project.
# For complete copyright and license terms please see the LICENSE at the root of this distribution.
#
# SPDX-License-Identifier: Apache-2.0 OR MIT
#
#

set(FILES
    Include/ROS2Sensors/ROS2SensorsTypeIds.h
    Include/ROS2Sensors/Sensor/SensorConfiguration.h
    Include/ROS2Sensors/Sensor/SensorConfigurationRequestBus.h
    Include/ROS2Sensors/Sensor/ROS2SensorComponentBase.h
    Include/ROS2Sensors/Sensor/SensorHelper.h
    Include/ROS2Sensors/Camera/CameraCalibrationRequestBus.h
    Include/ROS2Sensors/Camera/CameraPostProcessingRequestBus.h
    Include/ROS2Sensors/GNSS/GNSSPostProcessingRequestBus.h
    Include/ROS2Sensors/Lidar/ClassSegmentationBus.h
    Include/ROS2Sensors/Lidar/LidarRaycasterBus.h
    Include/ROS2Sensors/Lidar/LidarSystemBus.h
    Include/ROS2Sensors/Lidar/LidarRegistrarBus.h

    Include/ROS2Sensors/Configuration/ConfigurationBus.h
    Include/ROS2Sensors/Configuration/ImuSensorConfiguration.h
    Include/ROS2Sensors/Configuration/CameraSensorConfiguration.h
    Include/ROS2Sensors/Configuration/ROS2OdometryCovariance.h
    Include/ROS2Sensors/Configuration/ROS2WheelOdometryConfiguration.h
    Include/ROS2Sensors/Configuration/LidarSensorConfiguration.h
    Include/ROS2Sensors/Configuration/LidarTemplate.h
    Include/ROS2Sensors/Configuration/LidarTemplateUtils.h

)
