# Copyright (c) Contributors to the Open 3D Engine Project.
# For complete copyright and license terms please see the LICENSE at the root of this distribution.
#
# SPDX-License-Identifier: Apache-2.0 OR MIT

# Note that this does not find any ros2 package in particular, but determines whether a distro is sourced properly
# Can be extended to handle supported / unsupported distros
if (NOT DEFINED ENV{ROS_DISTRO} OR NOT DEFINED ENV{AMENT_PREFIX_PATH})
    message(WARNING "To build ROS2 Gem a ROS distribution needs to be sourced, but none detected")
    set(ROS2_FOUND FALSE)
    return()
endif()
message(STATUS "Ros Distro is \"$ENV{ROS_DISTRO}\"")
set(ROS2_FOUND TRUE)

# Resolve each requested ROS2 package and wrap it as a 3rdParty::ROS2::<pkg> target.
# The engine runs find_package for any 3rdParty dependency that has no target yet, which is what invokes this file.
# Prebuilt Gems declare their ROS 2 dependencies by the same names,
# so they get resolved here against the locally installed packages.
# Kept in sync with Code/ros2_target_depends.cmake, which does the same wrapping at the producer's configure.
# Note: Source builds do not notice the difference - their macros re-resolve at each configure.
foreach(_package ${ROS2_FIND_COMPONENTS})
    if(TARGET 3rdParty::ROS2::${_package})
        continue()
    endif()
    if(ROS2_FIND_REQUIRED_${_package})
        find_package(${_package} CONFIG REQUIRED)
    else()
        find_package(${_package} CONFIG QUIET)
        if(NOT ${_package}_FOUND)
            continue()
        endif()
    endif()
    include(${${_package}_DIR}/${_package}Config.cmake OPTIONAL)
    if(${${_package}_FOUND_AMENT_PACKAGE})
        add_library(3rdParty::ROS2::${_package} INTERFACE IMPORTED GLOBAL)
        target_link_libraries(3rdParty::ROS2::${_package} INTERFACE ${${_package}_TARGETS})
    else()
        message(FATAL_ERROR "Package ${_package} was found (${${_package}_DIR}), but package is not an Ament package.")
    endif()
endforeach()
