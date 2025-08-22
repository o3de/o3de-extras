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
    Source/Clock/ITimeSource.h
    Source/Clock/ROS2ClockSystemComponent.h
    Source/Clock/ROS2ClockSystemComponent.cpp
    Source/Clock/ROS2TimeSource.cpp
    Source/Clock/ROS2TimeSource.h
    Source/Clock/SimulationTimeSource.cpp
    Source/Clock/SimulationTimeSource.h
    Source/Clock/RealTimeSource.cpp
    Source/Clock/RealTimeSource.h
    Source/Frame/NamespaceConfiguration.cpp
    Source/Frame/ROS2FrameConfiguration.cpp
    Source/Frame/ROS2FrameSystemComponent.cpp
    Source/Frame/ROS2FrameSystemComponent.h
    Source/SimulationUtils/FollowingCameraConfiguration.cpp
    Source/SimulationUtils/FollowingCameraConfiguration.h
    Source/SimulationUtils/FollowingCameraComponent.cpp
    Source/SimulationUtils/FollowingCameraComponent.h
)

# optional, legacy features compilation
if (WITH_GAZEBO_MSGS)
        list(APPEND FILES
                Source/Spawner/ROS2SpawnerComponent.cpp
                Source/Spawner/ROS2SpawnerComponent.h
                Source/Spawner/ROS2SpawnPointComponent.cpp
                Source/Spawner/ROS2SpawnPointComponent.h
                Source/Spawner/ROS2SpawnerComponentController.cpp
                Source/Spawner/ROS2SpawnerComponentController.h
                Source/Spawner/ROS2SpawnPointComponentController.cpp
                Source/Spawner/ROS2SpawnPointComponentController.h
        )
endif ()
