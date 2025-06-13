# Copyright (c) Contributors to the Open 3D Engine Project.
# For complete copyright and license terms please see the LICENSE at the root of this distribution.
#
# SPDX-License-Identifier: Apache-2.0 OR MIT

o3de_gem_setup("ROS2")

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/Code")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} PARENT_SCOPE)

ly_add_external_target_path(${CMAKE_CURRENT_SOURCE_DIR}/3rdParty)

# Include ROS2 target function here so that every Project with this Gem can make use of it
# Currently only affects the Gem itself but this likely will change in the future
include(${CMAKE_CURRENT_LIST_DIR}/Code/ros2_target_depends.cmake)

add_subdirectory(Code)
