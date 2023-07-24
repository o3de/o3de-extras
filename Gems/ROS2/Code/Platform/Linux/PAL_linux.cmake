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
        PACKAGE_HASH fd7f34bf9e9d20320ff29ae74ea0fce8b9f7688dd3c1dbd4b4b8a68d31b7d2ee)
elseif(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "aarch64")
    ly_associate_package(PACKAGE_NAME sdformat-13.5.0-rev0-linux-aarch64
        TARGETS sdformat
        PACKAGE_HASH ca0fedaa00d82f6b211a2ed94101f2dafb11fa2732c3955443db78325abda2a6)
endif()
