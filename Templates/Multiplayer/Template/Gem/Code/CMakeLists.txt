#
# Copyright (c) Contributors to the Open 3D Engine Project. For complete copyright and license terms please see the LICENSE at the root of this distribution.
#
# SPDX-License-Identifier: Apache-2.0 OR MIT
#
#

o3de_pal_dir(pal_dir ${CMAKE_CURRENT_LIST_DIR}/Platform/${PAL_PLATFORM_NAME} "${gem_restricted_path}" "${gem_path}" "${gem_parent_relative_path}")

ly_add_target(
    NAME ${Name}.Client.Static STATIC
    NAMESPACE Gem
    FILES_CMAKE
        ${NameLower}_autogen_files.cmake
        ${NameLower}_files.cmake
        ${pal_dir}/${NameLower}_${PAL_PLATFORM_NAME_LOWERCASE}_files.cmake
    INCLUDE_DIRECTORIES
        PRIVATE
            Source
            .
        PUBLIC
            Include
    BUILD_DEPENDENCIES
        PUBLIC
            Gem::DebugDraw
            Gem::LyShine
            Gem::StartingPointInput
            Gem::EMotionFXStaticLib
            Gem::PhysX
            Gem::Multiplayer
        PRIVATE
            Gem::LmbrCentral.Static
            Gem::Multiplayer.Client.Static
            Gem::PhysX.Static
            Gem::DebugDraw.Static
            Gem::ImGui.Static
            Gem::LyShine.Static
            Gem::GameState.Static
    AUTOGEN_RULES
        *.AutoComponent.xml,AutoComponent_Header.jinja,$path/$fileprefix.AutoComponent.h
        *.AutoComponent.xml,AutoComponent_Source.jinja,$path/$fileprefix.AutoComponent.cpp
        *.AutoComponent.xml,AutoComponentTypes_Header.jinja,$path/AutoComponentTypes.h
        *.AutoComponent.xml,AutoComponentTypes_Source.jinja,$path/AutoComponentTypes.cpp
)

ly_add_target(
    NAME ${Name}.Server.Static STATIC
    NAMESPACE Gem
    FILES_CMAKE
        ${NameLower}_autogen_files.cmake
        ${NameLower}_files.cmake
        ${pal_dir}/${NameLower}_${PAL_PLATFORM_NAME_LOWERCASE}_files.cmake
    INCLUDE_DIRECTORIES
        PRIVATE
            Source
            .
        PUBLIC
            Include
    BUILD_DEPENDENCIES
        PUBLIC
            Gem::StartingPointInput
            Gem::EMotionFXStaticLib
            Gem::PhysX
            Gem::Multiplayer
        PRIVATE
            Gem::LmbrCentral.Static
            Gem::Multiplayer.Server.Static
            Gem::PhysX.Static
            Gem::GameState.Static
    AUTOGEN_RULES
        *.AutoComponent.xml,AutoComponent_Header.jinja,$path/$fileprefix.AutoComponent.h
        *.AutoComponent.xml,AutoComponent_Source.jinja,$path/$fileprefix.AutoComponent.cpp
        *.AutoComponent.xml,AutoComponentTypes_Header.jinja,$path/AutoComponentTypes.h
        *.AutoComponent.xml,AutoComponentTypes_Source.jinja,$path/AutoComponentTypes.cpp
)

ly_add_target(
    NAME ${Name}.Unified.Static STATIC
    NAMESPACE Gem
    FILES_CMAKE
        ${NameLower}_autogen_files.cmake
        ${NameLower}_files.cmake
        ${pal_dir}/${NameLower}_${PAL_PLATFORM_NAME_LOWERCASE}_files.cmake
    INCLUDE_DIRECTORIES
        PRIVATE
            Source
            .
        PUBLIC
            Include
    BUILD_DEPENDENCIES
        PUBLIC
            Gem::DebugDraw
            Gem::LyShine
            Gem::StartingPointInput
            Gem::EMotionFXStaticLib
            Gem::PhysX
            Gem::Multiplayer
        PRIVATE
            Gem::LmbrCentral.Static
            Gem::Multiplayer.Unified.Static
            Gem::PhysX.Static
            Gem::DebugDraw.Static
            Gem::ImGui.Static
            Gem::LyShine.Static
            Gem::GameState.Static
    AUTOGEN_RULES
        *.AutoComponent.xml,AutoComponent_Header.jinja,$path/$fileprefix.AutoComponent.h
        *.AutoComponent.xml,AutoComponent_Source.jinja,$path/$fileprefix.AutoComponent.cpp
        *.AutoComponent.xml,AutoComponentTypes_Header.jinja,$path/AutoComponentTypes.h
        *.AutoComponent.xml,AutoComponentTypes_Source.jinja,$path/AutoComponentTypes.cpp
)

ly_add_target(
    NAME ${Name}.Client ${PAL_TRAIT_MONOLITHIC_DRIVEN_MODULE_TYPE}
    NAMESPACE Gem
    FILES_CMAKE
        ${NameLower}_shared_files.cmake
        ../../${NameLower}_asset_files.cmake
    INCLUDE_DIRECTORIES
        PRIVATE
            Source
            .
        PUBLIC
            Include
    BUILD_DEPENDENCIES
        PRIVATE
            Gem::${Name}.Client.Static
            Gem::Atom_AtomBridge.Static
)

ly_add_target(
    NAME ${Name}.Server ${PAL_TRAIT_MONOLITHIC_DRIVEN_MODULE_TYPE}
    NAMESPACE Gem
    FILES_CMAKE
        ${NameLower}_shared_files.cmake
        ../../${NameLower}_asset_files.cmake
    INCLUDE_DIRECTORIES
        PRIVATE
            Source
            .
        PUBLIC
            Include
    BUILD_DEPENDENCIES
        PRIVATE
            Gem::${Name}.Server.Static
            Gem::Atom_AtomBridge.Static
)

ly_add_target(
    NAME ${Name} ${PAL_TRAIT_MONOLITHIC_DRIVEN_MODULE_TYPE}
    NAMESPACE Gem
    FILES_CMAKE
        ${NameLower}_shared_files.cmake
        ../../${NameLower}_asset_files.cmake
    INCLUDE_DIRECTORIES
        PRIVATE
            Source
            .
        PUBLIC
            Include
    BUILD_DEPENDENCIES
        PRIVATE
            Gem::${Name}.Unified.Static
            Gem::Atom_AtomBridge.Static
)

# if enabled, ${Name} is used by all kinds of applications
ly_create_alias(NAME ${Name}.Builders NAMESPACE Gem TARGETS Gem::${Name})
ly_create_alias(NAME ${Name}.Tools    NAMESPACE Gem TARGETS Gem::${Name})
ly_create_alias(NAME ${Name}.Clients  NAMESPACE Gem TARGETS Gem::${Name}.Client)
ly_create_alias(NAME ${Name}.Servers  NAMESPACE Gem TARGETS Gem::${Name}.Server)
ly_create_alias(NAME ${Name}.Unified  NAMESPACE Gem TARGETS Gem::${Name})

################################################################################
# Gem dependencies
################################################################################

# Maps the ${Name} Project with the specified list of enabled gems
ly_enable_gems(PROJECT_NAME ${Name} GEM_FILE enabled_gems.cmake)

# If we build a server, then add the project name to the list of server launcher projects
if(PAL_TRAIT_BUILD_SERVER_SUPPORTED)
    set_property(GLOBAL APPEND PROPERTY LY_LAUNCHER_SERVER_PROJECTS ${Name})
endif()

# If we build a server, then add the project name to the list of server launcher projects
if(PAL_TRAIT_BUILD_UNIFIED_SUPPORTED)
    set_property(GLOBAL APPEND PROPERTY LY_LAUNCHER_UNIFIED_PROJECTS ${Name})
endif()

set_property(TARGET ${Name} APPEND PROPERTY GAMELAUNCHER_ADDITIONAL_VS_DEBUGGER_COMMAND_ARGUMENTS "--console-command-file=\"launch_client.cfg\"")
set_property(TARGET ${Name} APPEND PROPERTY SERVERLAUNCHER_ADDITIONAL_VS_DEBUGGER_COMMAND_ARGUMENTS "--console-command-file=\"launch_server.cfg\"")
set_property(TARGET ${Name} APPEND PROPERTY UNIFIEDLAUNCHER_ADDITIONAL_VS_DEBUGGER_COMMAND_ARGUMENTS "--console-command-file=\"launch_server.cfg\"")