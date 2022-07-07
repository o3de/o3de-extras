#
# Copyright (c) Contributors to the Open 3D Engine Project.
# For complete copyright and license terms please see the LICENSE at the root of this distribution.
#
# SPDX-License-Identifier: Apache-2.0 OR MIT
#
#

set(FILES
    Include/OpenXRVk/OpenXRVkDevice.h
    Include/OpenXRVk/OpenXRVkInput.h
    Include/OpenXRVk/OpenXRVkInstance.h
    Include/OpenXRVk/OpenXRVkPhysicalDevice.h
    Include/OpenXRVk/OpenXRVkSession.h
    Include/OpenXRVk/OpenXRVkSpace.h
    Include/OpenXRVk/OpenXRVkSwapChain.h
    Include/OpenXRVk/OpenXRVkSystemComponent.h
    Include/OpenXRVk/OpenXRVkUtils.h
    Include/OpenXRVk/OpenXRVkFunctionLoader.h
    Include/OpenXRVk/OpenXRVkGladFunctionLoader.h
    Source/OpenXRVkDevice.cpp
    Source/OpenXRVkInput.cpp
    Source/OpenXRVkInstance.cpp
    Source/OpenXRVkPhysicalDevice.cpp
    Source/OpenXRVkSession.cpp
    Source/OpenXRVkSpace.cpp
    Source/OpenXRVkSwapChain.cpp
    Source/OpenXRVkSystemComponent.cpp
    Source/OpenXRVkUtils.cpp
    Source/OpenXRVkFunctionLoader.cpp
    Source/OpenXRVkGladFunctionLoader.cpp
)

set(SKIP_UNITY_BUILD_INCLUSION_FILES
    # The following file defines GLAD_VULKAN_IMPLEMENTATION before including vulkan.h changing
    # the behavior inside vulkan.h. Other files also includes vulkan.h so this file cannot
    # be added to unity, other files could end up including vulkan.h and making this one fail.
    Source/OpenXRVkGladFunctionLoader.cpp
)