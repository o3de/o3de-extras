#
# Copyright (c) Contributors to the Open 3D Engine Project.
# For complete copyright and license terms please see the LICENSE at the root of this distribution.
#
# SPDX-License-Identifier: Apache-2.0 OR MIT
#
#

o3de_pal_dir(pal_include_dir ${CMAKE_CURRENT_LIST_DIR}/Include/OpenXRVk/Platform/${PAL_PLATFORM_NAME} "${gem_restricted_path}" "${gem_path}" "${gem_parent_relative_path}")
o3de_pal_dir(pal_source_dir ${CMAKE_CURRENT_LIST_DIR}/Source/Platform/${PAL_PLATFORM_NAME} "${gem_restricted_path}" "${gem_path}" "${gem_parent_relative_path}")

include(${pal_source_dir}/PAL_${PAL_PLATFORM_NAME_LOWERCASE}.cmake)

if(PAL_TRAIT_OPENXRVK_TARGETS_ALREADY_DEFINED)
    return() # OpenXRVk targets already defined in PAL_${PAL_PLATFORM_NAME_LOWERCASE}.cmake
endif()

if(NOT PAL_TRAIT_OPENXRVK_SUPPORTED)

    # Create stub modules. Once we support gem loading configuration, we can remove this stubbed targets
    ly_add_target(
        NAME OpenXRVk ${PAL_TRAIT_MONOLITHIC_DRIVEN_MODULE_TYPE}
        NAMESPACE Gem
        FILES_CMAKE
            openxrvk_stub_module.cmake
        INCLUDE_DIRECTORIES
            PRIVATE
                Source
        BUILD_DEPENDENCIES
            PRIVATE
                AZ::AzCore
    )

    return() # Do not create the rest of the targets

endif()

ly_add_target(
    NAME OpenXRVk.Static STATIC
    NAMESPACE Gem
    FILES_CMAKE
        openxrvk_private_common_files.cmake
        ${pal_source_dir}/platform_private_${PAL_PLATFORM_NAME_LOWERCASE}_files.cmake
        ${pal_include_dir}/platform_private_${PAL_PLATFORM_NAME_LOWERCASE}_files.cmake
    PLATFORM_INCLUDE_FILES
        ${pal_source_dir}/platform_private_static_${PAL_PLATFORM_NAME_LOWERCASE}.cmake
    INCLUDE_DIRECTORIES
        PRIVATE
            Source
            ${pal_source_dir}
        PUBLIC
            Include
            ${pal_include_dir}
    BUILD_DEPENDENCIES
        PUBLIC
            AZ::AzCore
            AZ::AzFramework
            3rdParty::OpenXR
            3rdParty::glad_vulkan
            AZ::AtomCore
            Gem::Atom_RPI.Public
            Gem::XR
)

ly_add_target(
    NAME OpenXRVk ${PAL_TRAIT_MONOLITHIC_DRIVEN_MODULE_TYPE}
    NAMESPACE Gem
    FILES_CMAKE
        openxrvk_private_common_shared_files.cmake
    INCLUDE_DIRECTORIES
        PRIVATE
            Source
            ${pal_source_dir}
        PUBLIC
            Include
            ${pal_include_dir}
    BUILD_DEPENDENCIES
        PRIVATE
            Gem::OpenXRVk.Static
)

