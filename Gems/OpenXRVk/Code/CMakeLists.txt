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
            ${openxr_dependency}
            AZ::AtomCore
            Gem::Atom_RHI_Vulkan.Reflect
            Gem::Atom_RHI_Vulkan.Glad.Static
            Gem::XR.Static
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

# use the OpenXRVk module in all aliases:
ly_create_alias(NAME OpenXRVk.Clients NAMESPACE Gem TARGETS Gem::OpenXRVk)
ly_create_alias(NAME OpenXRVk.Unified NAMESPACE Gem TARGETS Gem::OpenXRVk)

if(PAL_TRAIT_BUILD_HOST_TOOLS)
    ly_create_alias(NAME OpenXRVk.Tools NAMESPACE Gem TARGETS Gem::OpenXRVk)
    ly_create_alias(NAME OpenXRVk.Builders NAMESPACE Gem TARGETS Gem::OpenXRVk)
endif()

################################################################################
# Tests
################################################################################
if(PAL_TRAIT_BUILD_TESTS_SUPPORTED)

    ly_add_target(
        NAME OpenXRVk.Tests ${PAL_TRAIT_TEST_TARGET_TYPE}
        NAMESPACE Gem
        OUTPUT_NAME OpenXRVk.Tests
        FILES_CMAKE
            openxrvk_tests_files.cmake
            ${pal_source_dir}/platform_test_${PAL_PLATFORM_NAME_LOWERCASE}_files.cmake
        INCLUDE_DIRECTORIES
            PRIVATE
                Tests
                .
                Source
                ${pal_source_dir}
        BUILD_DEPENDENCIES
            PRIVATE
                AZ::AzTestShared
                AZ::AzTest
                Gem::OpenXRVk.Static
    )
    ly_add_googletest(
        NAME Gem::OpenXRVk.Tests
    )

endif()