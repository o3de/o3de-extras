# Copyright (c) Contributors to the Open 3D Engine Project.
# For complete copyright and license terms please see the LICENSE at the root of this distribution.
#
# SPDX-License-Identifier: Apache-2.0 OR MIT

if (NOT "$ENV{ROS_DISTRO}" STREQUAL "${ROS_DISTRO}")
    message(FATAL_ERROR "ROS 2 distribution does not match. Configuration created for: ${ROS_DISTRO}, sourced distribution: $ENV{ROS_DISTRO}. Please reconfigure project")
endif()
