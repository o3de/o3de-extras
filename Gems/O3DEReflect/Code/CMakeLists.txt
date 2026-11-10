# Copyright (c) Contributors to the Open 3D Engine Project.
# For complete copyright and license terms please see the LICENSE at the root of this distribution.
#
# SPDX-License-Identifier: Apache-2.0 OR MIT

# O3DEReflect gem - no platform-specific code needed

ly_add_target(
    NAME O3DEReflect.Static STATIC
    NAMESPACE Gem
    FILES_CMAKE
        o3dereflect_files.cmake
    INCLUDE_DIRECTORIES
        PUBLIC
            Include
            Source
    BUILD_DEPENDENCIES
        PUBLIC
            AZ::AzCore
            AZ::AzFramework
)

ly_add_target(
    NAME O3DEReflect ${PAL_TRAIT_MONOLITHIC_DRIVEN_MODULE_TYPE}
    NAMESPACE Gem
    FILES_CMAKE
        o3dereflect_shared_files.cmake
    INCLUDE_DIRECTORIES
        PUBLIC
            Include
        PRIVATE
            Source
    BUILD_DEPENDENCIES
        PRIVATE
            Gem::O3DEReflect.Static
)

# Inject the gem name into the Module source file
ly_add_source_properties(
    SOURCES
        Source/O3DEReflectModule.cpp
    PROPERTY COMPILE_DEFINITIONS
        VALUES
            O3DE_GEM_NAME=O3DEReflect
            O3DE_GEM_VERSION=1.0.0
)

# Load the "Gem::O3DEReflect" module in all types of applications
ly_create_alias(NAME O3DEReflect.Servers  NAMESPACE Gem TARGETS Gem::O3DEReflect)
ly_create_alias(NAME O3DEReflect.Unified  NAMESPACE Gem TARGETS Gem::O3DEReflect)
ly_create_alias(NAME O3DEReflect.Clients  NAMESPACE Gem TARGETS Gem::O3DEReflect)
ly_create_alias(NAME O3DEReflect.Tools    NAMESPACE Gem TARGETS Gem::O3DEReflect)
ly_create_alias(NAME O3DEReflect.Builders NAMESPACE Gem TARGETS Gem::O3DEReflect)


