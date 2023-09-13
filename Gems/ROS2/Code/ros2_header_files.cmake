# Copyright (c) Contributors to the Open 3D Engine Project.
# For complete copyright and license terms please see the LICENSE at the root of this distribution.
#
# SPDX-License-Identifier: Apache-2.0 OR MIT

set(FILES
        Include/ROS2/Camera/CameraCalibrationRequestBus.h
        Include/ROS2/Camera/CameraPostProcessingRequestBus.h
        Include/ROS2/Clock/PhysicallyStableClock.h
        Include/ROS2/Clock/SimulationClock.h
        Include/ROS2/Communication/PublisherConfiguration.h
        Include/ROS2/Communication/TopicConfiguration.h
        Include/ROS2/Communication/QoS.h
        Include/ROS2/Frame/NamespaceConfiguration.h
        Include/ROS2/Frame/ROS2FrameComponent.h
        Include/ROS2/Frame/ROS2Transform.h
        Include/ROS2/Gripper/GripperRequestBus.h
        Include/ROS2/Manipulation/Controllers/JointsPositionControllerRequests.h
        Include/ROS2/Manipulation/JointInfo.h
        Include/ROS2/Manipulation/JointsManipulationRequests.h
        Include/ROS2/Manipulation/JointsTrajectoryRequests.h
        Include/ROS2/Manipulation/MotorizedJoints/JointMotorControllerComponent.h
        Include/ROS2/Manipulation/MotorizedJoints/JointMotorControllerConfiguration.h
        Include/ROS2/Manipulation/MotorizedJoints/ManualMotorControllerComponent.h
        Include/ROS2/Manipulation/MotorizedJoints/PidMotorControllerBus.h
        Include/ROS2/Manipulation/MotorizedJoints/PidMotorControllerComponent.h
        Include/ROS2/RobotControl/ControlConfiguration.h
        Include/ROS2/RobotControl/ControlSubscriptionHandler.h
        Include/ROS2/RobotImporter/SDFormatSensorImporterHook.h
        Include/ROS2/Lidar/LidarRaycasterBus.h
        Include/ROS2/Lidar/LidarSystemBus.h
        Include/ROS2/Lidar/LidarRegistrarBus.h
        Include/ROS2/ROS2Bus.h
        Include/ROS2/ROS2GemUtilities.h
        Include/ROS2/Sensor/Events/EventSourceAdapter.h
        Include/ROS2/Sensor/Events/SensorEventSource.h
        Include/ROS2/Sensor/Events/PhysicsBasedSource.h
        Include/ROS2/Sensor/Events/TickBasedSource.h
        Include/ROS2/Sensor/ROS2SensorComponent.h
        Include/ROS2/Sensor/ROS2SensorComponentBase.h
        Include/ROS2/Sensor/SensorConfiguration.h
        Include/ROS2/Spawner/SpawnerBus.h
        Include/ROS2/Utilities/Controllers/PidConfiguration.h
        Include/ROS2/Utilities/PhysicsCallbackHandler.h
        Include/ROS2/Utilities/ROS2Conversions.h
        Include/ROS2/Utilities/ROS2Names.h
        Include/ROS2/VehicleDynamics/VehicleInputControlBus.h
        )
