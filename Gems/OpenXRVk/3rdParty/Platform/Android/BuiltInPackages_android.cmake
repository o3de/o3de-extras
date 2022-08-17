#
# Copyright (c) Contributors to the Open 3D Engine Project.
# For complete copyright and license terms please see the LICENSE at the root of this distribution.
#
# SPDX-License-Identifier: Apache-2.0 OR MIT
#
#

set(ANDROID_USE_OCULUS_OPENXR OFF CACHE BOOL "When ON it uses OpenXR library from Oculus SDK.")

if(ANDROID_USE_OCULUS_OPENXR)
    include(${CMAKE_CURRENT_LIST_DIR}/FindOpenXROculus.cmake)
    set(openxr_dependency 3rdParty::OpenXROculus)
else()
    ly_associate_package(PACKAGE_NAME OpenXR-1.0.22-rev1-android    TARGETS OpenXR  PACKAGE_HASH 1227204583ce224c7e3843e82bb36deb576df6b458eecce46740cb8941902f21)
    set(openxr_dependency 3rdParty::OpenXR)
endif()
