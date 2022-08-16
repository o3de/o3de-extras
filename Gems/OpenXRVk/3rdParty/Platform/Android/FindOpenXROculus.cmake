#
# Copyright (c) Contributors to the Open 3D Engine Project.
# For complete copyright and license terms please see the LICENSE at the root of this distribution.
# 
# SPDX-License-Identifier: Apache-2.0 OR MIT
#
#

# this file actually ingests the library and defines targets.
set(TARGET_WITH_NAMESPACE "3rdParty::OpenXROculus")
if (TARGET ${TARGET_WITH_NAMESPACE})
    return()
endif()

set(MY_NAME "OpenXROculus")

get_property(openxrvk_gem_root GLOBAL PROPERTY "@GEMROOT:OpenXRVk@")

set(OculusOpenXRSDKPath ${openxrvk_gem_root}/External/OculusOpenXRMobileSDK)

set(${MY_NAME}_INCLUDE_DIR 
    ${OculusOpenXRSDKPath}/3rdParty/khronos/openxr/OpenXR-SDK/include
    ${OculusOpenXRSDKPath}/OpenXR/Include)

set(PATH_TO_SHARED_LIBS ${OculusOpenXRSDKPath}/OpenXR/Libs/Android/arm64-v8a)

if(NOT EXISTS ${PATH_TO_SHARED_LIBS}/Release/libopenxr_loader.so)
    message(FATAL_ERROR
        "Oculus OpenXR loader library not found at ${PATH_TO_SHARED_LIBS}/Release. "
        "Oculus OpenXR Mobile SDK needs to be downloaded via https://developer.oculus.com/downloads/native-android/ "
        "and uncompressed into OpenXRVk/External/OculusOpenXRMobileSDK folder.")
    return()
endif()

add_library(${TARGET_WITH_NAMESPACE} SHARED IMPORTED GLOBAL)
ly_target_include_system_directories(TARGET ${TARGET_WITH_NAMESPACE} INTERFACE ${${MY_NAME}_INCLUDE_DIR})
set_target_properties(${TARGET_WITH_NAMESPACE}
    PROPERTIES
        IMPORTED_LOCATION ${PATH_TO_SHARED_LIBS}/Release/libopenxr_loader.so
        IMPORTED_LOCATION_DEBUG ${PATH_TO_SHARED_LIBS}/Debug/libopenxr_loader.so)

set(${MY_NAME}_FOUND True)
