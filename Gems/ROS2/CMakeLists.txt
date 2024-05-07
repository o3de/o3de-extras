# Copyright (c) Contributors to the Open 3D Engine Project.
# For complete copyright and license terms please see the LICENSE at the root of this distribution.
#
# SPDX-License-Identifier: Apache-2.0 OR MIT

# Query the gem name from the gem.json file if possible
# otherwise fallback to using ${Name}
o3de_find_ancestor_gem_root(gempath gem_name "${CMAKE_CURRENT_SOURCE_DIR}")
if (NOT gem_name)
    set(gem_name "ROS2")
endif()

# Add pip requirements required for ros2 packages.
update_pip_requirements(${CMAKE_CURRENT_LIST_DIR}/requirements.txt ROS2)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/Code")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} PARENT_SCOPE)

set(gem_path ${CMAKE_CURRENT_LIST_DIR})
set(gem_json ${gem_path}/gem.json)
o3de_restricted_path(${gem_json} gem_restricted_path gem_parent_relative_path)

ly_add_external_target_path(${CMAKE_CURRENT_LIST_DIR}/3rdParty)

# Include ROS2 target function here so that every Project with this Gem can make use of it
# Currently only affects the Gem itself but this likely will change in the future
include(${CMAKE_CURRENT_LIST_DIR}/Code/ros2_target_depends.cmake)

add_subdirectory(Code)
