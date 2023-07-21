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
        PACKAGE_HASH 407a335c1fefc134e89b2dc2d1ffd6a47ea33546e77df4aaabf3edf8b3402ecb)
elseif(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "aarch64")
    ly_associate_package(PACKAGE_NAME sdformat-13.5.0-rev0-linux-aarch64
        TARGETS sdformat
        PACKAGE_HASH a23977b262d6d80a5e1b22c24db27d25fe827c982296766c0f04ebf77ad0a217)
endif()
