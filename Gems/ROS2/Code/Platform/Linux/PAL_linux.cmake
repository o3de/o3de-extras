#
# Copyright (c) Contributors to the Open 3D Engine Project.
# For complete copyright and license terms please see the LICENSE at the root of this distribution.
#
# SPDX-License-Identifier: Apache-2.0 OR MIT
#
#

set(PAL_TRAIT_BUILD_ROS2_GEM_SUPPORTED TRUE)

if(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "x86_64")
    ly_associate_package(PACKAGE_NAME sdformat-13.5.0-rev2-linux
        TARGETS sdformat
        PACKAGE_HASH b8e988954c07f41b99ba0950da3f4d3e8489ffabaaff157f79ab0c716e2142e0)
elseif(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "aarch64")
    ly_associate_package(PACKAGE_NAME sdformat-13.5.0-rev2-linux-aarch64
        TARGETS sdformat
        PACKAGE_HASH 7e51cc60c61a058c1f8aba7277574946ab974af3ff4601884e72380e8585c0ea)
endif()
