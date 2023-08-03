#
# Copyright (c) Contributors to the Open 3D Engine Project.
# For complete copyright and license terms please see the LICENSE at the root of this distribution.
#
# SPDX-License-Identifier: Apache-2.0 OR MIT
#
#

set(PAL_TRAIT_BUILD_ROS2_GEM_SUPPORTED TRUE)

if(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "x86_64")
    ly_associate_package(PACKAGE_NAME sdformat-13.5.0-rev1-linux
        TARGETS sdformat
        PACKAGE_HASH 71a86e255cfc7a176f6087e069fb26c9837c98e277becf87f3a4b0e25fe90bd3)
elseif(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "aarch64")
    ly_associate_package(PACKAGE_NAME sdformat-13.5.0-rev1-linux-aarch64
        TARGETS sdformat
        PACKAGE_HASH c8ca2ffad604162cbc8ed852c8ea8695851c4c69c5f39f2a9d8e423511ce78f9)
endif()
