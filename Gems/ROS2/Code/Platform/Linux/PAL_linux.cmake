#
# Copyright (c) Contributors to the Open 3D Engine Project.
# For complete copyright and license terms please see the LICENSE at the root of this distribution.
#
# SPDX-License-Identifier: Apache-2.0 OR MIT
#
#

set(PAL_TRAIT_BUILD_ROS2_GEM_SUPPORTED TRUE)

if(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "x86_64")
    ly_associate_package(PACKAGE_NAME sdformat-13.5.0-rev0-linux
        TARGETS sdformat
        PACKAGE_HASH 4a2ae3ababc253e080440696b42930bf18cb350e3e0d1733e00204894602bc7f)
elseif(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "aarch64")
    ly_associate_package(PACKAGE_NAME sdformat-13.5.0-rev0-linux-aarch64
        TARGETS sdformat
        PACKAGE_HASH 5411d678bd5143fc4972d53fc0a8858aa8d05b5a83459c88827637dbe7afc230)
endif()
