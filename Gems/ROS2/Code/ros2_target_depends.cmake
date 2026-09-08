# Copyright (c) Contributors to the Open 3D Engine Project.
# For complete copyright and license terms please see the LICENSE at the root of this distribution.
#
# SPDX-License-Identifier: Apache-2.0 OR MIT

# Record dependencies on ROS 2 packages through O3DE's native model:
# wrap as a 3rdParty::ROS2::<pkg> target and link with ly_target_link_libraries.
# As a result Install_common.cmake can see those.
# Prebuilt Gems can provide these names in their generated cmake;
# consumers re-resolve them  against the locally installed packages via 3rdParty/FindROS2.cmake.
# Kept in sync with 3rdParty/FindROS2.cmake, which does the same wrapping at the # consumer's configure. 
function(_ros2_wrap_ament_package _package)
    if (TARGET 3rdParty::ROS2::${_package})
        return()
    endif ()
    include(${${_package}_DIR}/${_package}Config.cmake OPTIONAL)
    if (${${_package}_FOUND_AMENT_PACKAGE})
        message(DEBUG "Package ${_package} was found (${${_package}_DIR}) version ${${_package}_VERSION} targets : ${${_package}_TARGETS}")
        add_library(3rdParty::ROS2::${_package} INTERFACE IMPORTED GLOBAL)
        target_link_libraries(3rdParty::ROS2::${_package} INTERFACE ${${_package}_TARGETS})
    else ()
        message(FATAL_ERROR "Package ${_package} was found (${${_package}_DIR}), but package is not an Ament package.")
    endif ()
endfunction()

function(target_depends_on_ros2_package TARGET_NAME)
    list(GET ARGN 0 _package)
    find_package(${ARGN})
    if (${${_package}_FOUND})
        _ros2_wrap_ament_package(${_package})
        ly_target_link_libraries(${TARGET_NAME} PUBLIC 3rdParty::ROS2::${_package})
    else ()
        message(DEBUG "Package ${_package} was not found.")
    endif ()
endfunction()

function(target_depends_on_ros2_packages TARGET_NAME)
    foreach (_package IN LISTS ARGN)
        target_depends_on_ros2_package(${TARGET_NAME} ${_package} REQUIRED)
    endforeach ()
endfunction()

function(target_interface_depends_on_ros2_package TARGET_NAME)
    list(GET ARGN 0 _package)
    find_package(${ARGN})
    _ros2_wrap_ament_package(${_package})
    ly_target_link_libraries(${TARGET_NAME} INTERFACE 3rdParty::ROS2::${_package})
endfunction()

function(target_interface_depends_on_ros2_packages TARGET_NAME)
    foreach (_package IN LISTS ARGN)
        target_interface_depends_on_ros2_package(${TARGET_NAME} ${_package} REQUIRED)
    endforeach ()
endfunction()
