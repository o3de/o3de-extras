#
# Copyright (c) Contributors to the Open 3D Engine Project.
# For complete copyright and license terms please see the LICENSE at the root of this distribution.
#
# SPDX-License-Identifier: Apache-2.0 OR MIT
#
#

set(PAL_TRAIT_ROS2ROBOTIMPORTER_SUPPORTED TRUE)
set(PAL_TRAIT_ROS2ROBOTIMPORTER_TEST_SUPPORTED FALSE)
set(PAL_TRAIT_ROS2ROBOTIMPORTER_EDITOR_TEST_SUPPORTED TRUE)

if(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "x86_64")
    ly_associate_package(PACKAGE_NAME sdformat-16.0.1-rev1-linux
        TARGETS sdformat
        PACKAGE_HASH 41125211ce8f96c2985eca1181e2a5e820fa045511546af0f7a5f60669dd5ebf)
elseif(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "aarch64")
    ly_associate_package(PACKAGE_NAME sdformat-16.0.1-rev1-linux-aarch64
        TARGETS sdformat
        PACKAGE_HASH a4893bd1586fd7ceee93ae5e3febb8c95fe868cd053b44e4b0750d47968495c2)
endif()
