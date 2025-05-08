# Copyright (c) Contributors to the Open 3D Engine Project.
# For complete copyright and license terms please see the LICENSE at the root of this distribution.
#
# SPDX-License-Identifier: Apache-2.0 OR MIT

set(FILES
        ../Assets/Passes/PipelineRenderToTextureROSColor.pass
        ../Assets/Passes/PipelineRenderToTextureROSDepth.pass
        ../Assets/Passes/PipelineROSColor.pass
        ../Assets/Passes/PipelineROSDepth.pass
        ../Assets/Passes/ROSPassTemplates.azasset
        Source/Clock/ROS2Clock.cpp
        Source/Clock/ROS2TimeSource.cpp
        Source/Clock/SimulationTimeSource.cpp
        Source/Clock/RealTimeSource.cpp
        Source/Communication/QoS.cpp
        Source/Communication/PublisherConfiguration.cpp
        Source/Communication/TopicConfiguration.cpp
        Source/Frame/NamespaceConfiguration.cpp
        Source/Frame/ROS2FrameComponent.cpp
        Source/Frame/ROS2FrameConfiguration.cpp
        Source/Frame/ROS2Transform.cpp
        Source/ROS2ModuleInterface.h
        Source/Sensor/Events/PhysicsBasedSource.cpp
        Source/Sensor/Events/TickBasedSource.cpp
        Source/SimulationUtils/FollowingCameraConfiguration.cpp
        Source/SimulationUtils/FollowingCameraConfiguration.h
        Source/SimulationUtils/FollowingCameraComponent.cpp
        Source/SimulationUtils/FollowingCameraComponent.h
        Source/Spawner/ROS2SpawnerComponent.cpp
        Source/Spawner/ROS2SpawnerComponent.h
        Source/Spawner/ROS2SpawnPointComponent.cpp
        Source/Spawner/ROS2SpawnPointComponent.h
        Source/Spawner/ROS2SpawnerComponentController.cpp
        Source/Spawner/ROS2SpawnerComponentController.h
        Source/Spawner/ROS2SpawnPointComponentController.cpp
        Source/Spawner/ROS2SpawnPointComponentController.h
        Source/SystemComponents/ROS2SystemComponent.cpp
        Source/SystemComponents/ROS2SystemComponent.h
        Source/Utilities/ROS2Conversions.cpp
        Source/Utilities/ROS2Names.cpp
)
