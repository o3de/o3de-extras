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
        PACKAGE_HASH 6bd52db178a5bfed12555db2a9710bb4f20baab73f0d7374c6c70b6568a3ba3f)
elseif(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "aarch64")
    ly_associate_package(PACKAGE_NAME sdformat-13.5.0-rev0-linux-aarch64
        TARGETS sdformat
        PACKAGE_HASH f422699e67a92a787536c2f037f5f0e0904d82f88e66f2a1b856409aec67b0fe)
endif()
