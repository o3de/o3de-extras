# Copyright (c) Contributors to the Open 3D Engine Project.
# For complete copyright and license terms please see the LICENSE at the root of this distribution.
#
# SPDX-License-Identifier: Apache-2.0 OR MIT

function(target_depends_on_ros2_packages TARGET_NAME)
    find_package(ROS2 MODULE)
    if (NOT ROS2_FOUND)
        return()
    endif()
    message(DEBUG "Building ROS2 Gem with ros2 $ENV{ROS_DISTRO}")
    set(_ament_prefix_path "$ENV{AMENT_PREFIX_PATH}")

    # ros2 directories with libraries, e.g. /opt/ros/galactic/lib, locally built custom packages etc.
    set(_ros2_library_directories)
    set(_ros2_include_directories)
    set(_ros2_package_libraries)
    foreach(_ros2_packages_path IN LISTS _ament_prefix_path)
        string(REPLACE ":" ";" _ros2_packages_path ${_ros2_packages_path})
        list(APPEND _ros2_library_directories "${_ros2_packages_path}/lib")
        list(APPEND _ros2_include_directories "${_ros2_packages_path}/include")
    endforeach()
    foreach(_package IN LISTS ARGN)
        message(DEBUG "Processing package: ${_package}")
        find_package(${_package} REQUIRED)
        list(APPEND _ros2_package_libraries "${${_package}_LIBRARIES}")
        list(APPEND _ros2_include_directories "${${_package}_INCLUDE_DIRS}")
    endforeach()
    target_include_directories(${TARGET_NAME} PUBLIC ${_ros2_include_directories})
    target_link_libraries(${TARGET_NAME} PUBLIC ${_ros2_package_libraries})
endfunction()
