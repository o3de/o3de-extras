# Copyright (c) Contributors to the Open 3D Engine Project.
# For complete copyright and license terms please see the LICENSE at the root of this distribution.
#
# SPDX-License-Identifier: Apache-2.0 OR MIT

set(FILES
    Source/Frame/ROS2FrameSystemComponent.cpp
    Source/Frame/ROS2FrameSystemComponent.h
    Source/Frame/ROS2FrameSystemBus.h
    Source/Tools/ROS2EditorSystemComponent.cpp
    Source/Tools/ROS2EditorSystemComponent.h
)

# optional, legacy features compilation
if (WITH_GAZEBO_MSGS)
    list(APPEND FILES
        Source/Spawner/ROS2SpawnerEditorComponent.cpp
        Source/Spawner/ROS2SpawnerEditorComponent.h
        Source/Spawner/ROS2SpawnPointEditorComponent.cpp
        Source/Spawner/ROS2SpawnPointEditorComponent.h
    )
endif ()
