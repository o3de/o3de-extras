#
# Copyright (c) Contributors to the Open 3D Engine Project.
# For complete copyright and license terms please see the LICENSE at the root of this distribution.
#
# SPDX-License-Identifier: Apache-2.0 OR MIT
#
#

# Expose some internal of Optick for better integration with 03DE
set(OPTICK_GIT_PATCH "${CMAKE_CURRENT_SOURCE_DIR}/optick.patch")

# Add Optick dependency using FetchContent
include(FetchContent)
FetchContent_Declare(
    Optick
    GIT_REPOSITORY "https://github.com/bombomby/optick.git"
    GIT_TAG "8abd28dee1a4034c973a3d32cd1777118e72df7e" # version 1.4.0+
    PATCH_COMMAND cmake -P "${LY_ROOT_FOLDER}/cmake/PatchIfNotAlreadyPatched.cmake" ${OPTICK_GIT_PATCH}
)

set(OPTICK_ENABLED ON)
set(OPTICK_BUILD_GUI_APP OFF)
set(OPTICK_INSTALL_TARGETS OFF)

message(STATUS "OptickProfiler Gem uses https://github.com/bombomby/optick.git version 1.4.0 (License: MIT)")
message(STATUS "      With patch: ${OPTICK_GIT_PATCH}")

FetchContent_MakeAvailable(Optick)

# Let's not clutter the root of any IDE folder structure with 3rd party dependencies
# Setting the FOLDER makes it show up there in the solution build in VS and similarly
# any other IDEs that organize in folders.
set_target_properties(
        OptickCore 
    PROPERTIES 
        FOLDER "3rdParty Dependencies"
)

ly_add_target(
    NAME ${gem_name}.Static STATIC
    NAMESPACE Gem
    FILES_CMAKE
        optickprofiler_files.cmake
    INCLUDE_DIRECTORIES
        PUBLIC
            Include
        PRIVATE
            Source
    BUILD_DEPENDENCIES
        PUBLIC
            AZ::AzCore
            AZ::AzFramework
            OptickCore
)

ly_add_target(
    NAME ${gem_name} ${PAL_TRAIT_MONOLITHIC_DRIVEN_MODULE_TYPE}
    NAMESPACE Gem
    FILES_CMAKE
        optickprofiler_shared_files.cmake
    INCLUDE_DIRECTORIES
        PUBLIC
            Include
        PRIVATE
            Source
    BUILD_DEPENDENCIES
        PRIVATE
            Gem::${gem_name}.Static
)

ly_add_source_properties(
    SOURCES
        Source/ProfilerModule.cpp
    PROPERTY COMPILE_DEFINITIONS
        VALUES
            O3DE_GEM_NAME=${gem_name}
            O3DE_GEM_VERSION=${gem_version})

ly_create_alias(NAME ${gem_name}.Servers NAMESPACE Gem TARGETS Gem::${gem_name})
ly_create_alias(NAME ${gem_name}.Builders NAMESPACE Gem TARGETS Gem::${gem_name})
ly_create_alias(NAME ${gem_name}.Clients NAMESPACE Gem TARGETS Gem::${gem_name})
ly_create_alias(NAME ${gem_name}.Unified NAMESPACE Gem TARGETS Gem::${gem_name})
ly_create_alias(NAME ${gem_name}.Tools NAMESPACE Gem TARGETS Gem::${gem_name})
