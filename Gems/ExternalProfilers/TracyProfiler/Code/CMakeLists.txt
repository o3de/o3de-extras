#
# Copyright (c) Contributors to the Open 3D Engine Project.
# For complete copyright and license terms please see the LICENSE at the root of this distribution.
#
# SPDX-License-Identifier: Apache-2.0 OR MIT
#
#

# Add Tracy dependency using FetchContent
include(FetchContent)
FetchContent_Declare(
    Tracy
    GIT_REPOSITORY "https://github.com/wolfpld/tracy"
    GIT_TAG "5d542dc09f3d9378d005092a4ad446bd405f819a" # version 0.11.1
)

set(TRACY_ENABLE ON)
set(TRACY_CALLSTACK ON)
set(TRACY_ON_DEMAND ON)
ly_append_configurations_options(
    DEFINES
        TRACY_ENABLE
)

FetchContent_MakeAvailable(Tracy)

# Let's not clutter the root of any IDE folder structure with 3rd party dependencies
# Setting the FOLDER makes it show up there in the solution build in VS and similarly
# any other IDEs that organize in folders.
set_target_properties(
        TracyClient 
    PROPERTIES 
        FOLDER "3rdParty Dependencies"
)

ly_add_target(
    NAME ${gem_name}.Static STATIC
    NAMESPACE Gem
    FILES_CMAKE
        tracyprofiler_files.cmake
    INCLUDE_DIRECTORIES
        PUBLIC
            Include
        PRIVATE
            Source
            ${Tracy_SOURCE_DIR}/public/tracy
    BUILD_DEPENDENCIES
        PUBLIC
            AZ::AzCore
            AZ::AzFramework
            TracyClient
)

ly_add_target(
    NAME ${gem_name} ${PAL_TRAIT_MONOLITHIC_DRIVEN_MODULE_TYPE}
    NAMESPACE Gem
    FILES_CMAKE
        tracyprofiler_shared_files.cmake
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
